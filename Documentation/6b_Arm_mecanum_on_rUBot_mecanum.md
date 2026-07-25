# Arm Mecanum on the rUBot Mecanum Platform

## Overview

The rUBot mecanum platform can be equipped with a servo arm to form a **mobile manipulator**.

A mobile manipulator combines:

```text
Mobile base
+
Robot arm
```

The mobile base provides movement in the environment, while the arm provides local manipulation relative to the robot platform.

The complete robot contains two independent ROS 2 control chains:

- one for the mecanum base;
- one for the servo arm.

The arm uses a simplified custom ROS 2 driver adapted from the main concepts of the `ros2_control` architecture.

---

## Mobile Manipulator Concept

The mecanum base controls the global motion of the robot:

```text
x
y
yaw
```

The arm controls the local configuration of the manipulator:

```text
arm_joint1
arm_joint2
arm_joint3
arm_joint4
arm_joint5
gripper
```

The two systems can receive commands independently and can operate at the same time.

```text
                       rUBot mobile manipulator

        MOBILE BASE                              ROBOT ARM

Navigation / teleoperation               Motion / kinematics node
             ↓                                      ↓
          /cmd_vel                    /arm_controller/joint_trajectory
             ↓                                      ↓
   mecanum base driver                 serial_trajectory_bridge_node
             ↓                                      ↓
      Arduino / motors                  Arduino Nano ESP32
             ↓                                      ↓
     mecanum wheels                     SG90 servos
```

---

## Complete Robot Architecture

The complete robot is composed of two independent subsystems connected through the ROS 2 model and TF tree.

```text
                           ROS 2 system

        ┌──────────────────────┴──────────────────────┐
        │                                             │
        ▼                                             ▼

   Mecanum base                                  Servo arm

   /cmd_vel                                      /arm_controller/joint_trajectory
        ↓                                             ↓
   base driver                                serial_trajectory_bridge_node
        ↓                                             ↓
   Arduino + motor control                    Arduino firmware
        ↓                                             ↓
   mecanum wheels                             SG90 servos
```

The base and the arm use different command topics, different drivers and different low-level hardware.

---

## Mobile Base Command Path

The mobile base receives velocity commands through:

```text
Topic:
/cmd_vel

Message:
geometry_msgs/msg/Twist
```

The command path is:

```text
Navigation or teleoperation node
                ↓
             /cmd_vel
                ↓
      mecanum base driver
                ↓
      Arduino motor controller
                ↓
       four mecanum wheels
```

The base driver converts the desired robot velocity into individual wheel commands.

---

## Mobile Base State Path

The mobile base can provide measured wheel and odometry information.

```text
Wheel encoders
      ↓
Arduino
      ↓
mecanum base driver
      ↓
/odom
/joint_states
TF: odom → base_footprint
```

Typical base outputs are:

```text
Topic:
/odom

Message:
nav_msgs/msg/Odometry
```

and wheel-joint states through:

```text
Topic:
/joint_states

Message:
sensor_msgs/msg/JointState
```

The wheel encoders provide measured information from the physical robot.

---

## Robot Arm Command Path

The robot arm receives trajectory commands through:

```text
Topic:
/arm_controller/joint_trajectory

Message:
trajectory_msgs/msg/JointTrajectory
```

The command path is:

```text
Motion, GUI or kinematics node
              ↓
       JointTrajectory
              ↓
/arm_controller/joint_trajectory
              ↓
serial_trajectory_bridge_node
              ↓
USB serial communication
              ↓
Arduino Nano ESP32
              ↓
Servo.write()
              ↓
SG90 servos
              ↓
Physical arm joints and gripper
```

The `serial_trajectory_bridge_node`:

- receives the trajectory;
- executes the trajectory points sequentially;
- converts ROS joint positions from radians to servo angles;
- applies servo centres, direction signs and limits;
- sends the servo commands through the serial port;
- publishes the last commanded joint positions.

The internal arm-driver architecture is described in:

```text
Documentation/5_servo_arm_driver.md
```

---

## Robot Arm State Path

The SG90 servos do not return their physical position to ROS 2.

The current arm state path is:

```text
Last commanded arm positions
              ↓
serial_trajectory_bridge_node
              ↓
/joint_states
```

The arm publishes:

```text
Topic:
/joint_states

Message:
sensor_msgs/msg/JointState
```

These values represent the expected arm configuration, not measured servo positions.

This is different from the mobile base, where wheel encoders can provide measured feedback.

---

## URDF and TF Integration

The arm must be integrated into the complete robot model.

The arm base is connected to the mecanum `base_link` using a fixed joint:

```xml
<joint name="arm_mount_joint" type="fixed">
```

This joint defines:

- the position of the arm on the platform;
- the mounting height;
- the arm orientation;
- the offset from the centre of the mobile base.

A simplified TF tree is:

```text
odom
  ↓
base_footprint
  ↓
base_link
  ↓
arm_base_link
  ↓
arm_joint1
  ↓
arm_link1
  ↓
...
  ↓
gripper / tool
```

When the mobile base moves, the complete arm TF tree moves with it.

---

## Base Motion and Arm Motion

The base and the arm operate in different reference frames.

### Base motion

The base moves the complete robot in the environment.

Typical commands are:

```text
move forward
move sideways
rotate
```

The base pose is normally represented relative to:

```text
odom
```

or:

```text
map
```

### Arm motion

The arm moves the wrist, gripper or TCP relative to:

```text
base_link
```

or:

```text
arm_base_link
```

Typical commands are:

```text
move the wrist forward
raise the arm
rotate the wrist
open or close the gripper
```

### Combined pose

The TCP pose in the environment depends on both the mobile-base pose and the arm configuration.

Conceptually:

```text
T_world_tcp
=
T_world_base
·
T_base_arm
·
T_arm_tcp
```

Therefore, moving the mobile base also changes the global pose of the arm TCP.

---

## Shared Joint-State Topic

The mobile base and the arm may both publish to:

```text
/joint_states
```

This is valid if they publish different joint names.

Example base joints:

```text
front_left_wheel_joint
front_right_wheel_joint
rear_left_wheel_joint
rear_right_wheel_joint
```

Example arm joints:

```text
arm_joint1
arm_joint2
arm_joint3
arm_joint4
arm_joint5
arm_joint6
```

`robot_state_publisher` uses these messages to update the joints of the complete robot model.

The integration must avoid:

- duplicated joint names;
- two nodes publishing different values for the same joint;
- inconsistent timestamps;
- joint names that do not match the URDF.

---

## Independent and Simultaneous Control

The base and arm use independent ROS 2 interfaces:

```text
Base command:
/cmd_vel

Arm command:
/arm_controller/joint_trajectory
```

Therefore, the robot can receive base and arm commands at the same time.

For example:

```text
Base:
move slowly forward

Arm:
move the gripper to a target configuration
```

However, the current system does not provide coordinated whole-body planning.

It does not automatically:

- coordinate the base trajectory with the arm trajectory;
- check collisions between the arm and the environment;
- optimise the combined base-arm motion;
- generate a single trajectory for the complete mobile manipulator.

The current architecture provides independent control of both subsystems.

---

## Complete Bringup

The complete robot bringup should start the base driver, arm driver, robot model and visualisation.

```text
my_robot_bringup
│
├── mecanum base driver
│     ├── subscribes to /cmd_vel
│     ├── publishes /odom
│     └── publishes wheel joint states
│
├── serial_trajectory_bridge_node
│     ├── subscribes to /arm_controller/joint_trajectory
│     └── publishes arm joint states
│
├── robot_state_publisher
│     └── publishes the complete TF tree
│
└── RViz
      └── displays the base, arm and gripper
```

This architecture allows the complete mobile manipulator to be represented and controlled within the same ROS 2 system.

---

## Basic Test Sequence

### 1. Start the complete robot bringup

```bash
ros2 launch my_robot_bringup my_robot_nano_bringup_hw.launch.py
```

The exact launch file may depend on the final integration of the arm into the mecanum repository.

### 2. Verify the mobile-base command topic

```bash
ros2 topic info /cmd_vel
```

### 3. Verify the arm command topic

```bash
ros2 topic info /arm_controller/joint_trajectory
```

### 4. Verify the complete joint-state topic

```bash
ros2 topic echo /joint_states
```

The output should contain wheel-joint names and arm-joint names.

### 5. Verify the TF tree

```bash
ros2 run tf2_tools view_frames
```

The generated TF tree should connect:

```text
odom
→ base_footprint
→ base_link
→ arm_base_link
→ arm links
→ gripper
```

### 6. Test the base

Publish a small velocity command or use the teleoperation node.

### 7. Test the arm

Run the joint slider GUI or send a simple joint target.

For the first hardware tests:

- keep the robot stationary;
- keep the arm clear of obstacles;
- move one subsystem at a time;
- use small velocities and small joint changes.

---

## Current Limitations

The current mobile manipulator has the following limitations:

- the base and arm are controlled independently;
- there is no coordinated whole-body planner;
- the SG90 servos do not provide measured position feedback;
- the arm `/joint_states` contain commanded positions;
- direct GUI control does not perform collision checking;
- the gripper is controlled as the sixth command of the arm driver;
- the base and arm use separate low-level control chains;
- simultaneous commands are possible, but they are not automatically synchronised.

These simplifications are appropriate for an educational mobile manipulator and make the architecture easier to understand and test.
