# ROS 2 Driver for the 6-DOF Mecanum Robot Arm

## Overview

This package provides a ROS 2 driver for the 6-DOF servo arm mounted on the rUBot mecanum platform.

The arm is driven by six SG90 servos controlled by an Arduino Nano ESP32.

The driver implements a standard ROS trajectory interface, allowing the arm to be controlled by motion nodes, kinematics nodes, or MoveIt2.

```text
Motion Nodes
     │
     ▼
/arm_controller/joint_trajectory
     │
     ▼
serial_trajectory_bridge_node
     │
 USB Serial
     ▼
Arduino Nano ESP32
     │
     ▼
6 SG90 Servos
```

The driver also publishes:

```text
/joint_states
```

so that RViz and robot_state_publisher can display the current arm configuration.

---

# Driver Node

Node:

```text
serial_trajectory_bridge_node
```

### Subscription

```text
/arm_controller/joint_trajectory
```

Type:

```text
trajectory_msgs/msg/JointTrajectory
```

### Publication

```text
/joint_states
```

Type:

```text
sensor_msgs/msg/JointState
```

---

# Driver Operation

The driver:

1. Receives a JointTrajectory message.
2. Executes each trajectory point sequentially.
3. Converts joint positions from radians to servo angles.
4. Applies servo offsets and sign corrections.
5. Sends six angles through the serial port.
6. Updates and publishes `/joint_states`.

Example serial command:

```text
90,120,45,90,90,90
```

---

# Launch Driver

```bash
ros2 launch my_arm_driver serial_trajectory_bridge.launch.py \
    serial_port:=/dev/ttyUSB0
```

Expected output:

```text
Serial trajectory bridge started on /dev/ttyUSB0 at 115200 baud
```

---

# Test 1: Verify Joint States

Launch the driver:

```bash
ros2 launch my_arm_driver serial_trajectory_bridge.launch.py
```

In another terminal:

```bash
ros2 topic echo /joint_states
```

Expected:

```yaml
name:
- arm_joint1
- arm_joint2
- arm_joint3
- arm_joint4
- arm_joint5
- arm_joint6

position:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
```

---

# Test 2: Single Target

Launch the driver:

```bash
ros2 launch my_arm_driver serial_trajectory_bridge.launch.py
```

Send a target:

```bash
ros2 run my_arm_driver send_joint_target_node \
--ros-args \
-p target_joints_deg:="[0,0,0,0,0,0]"
```

Example:

```bash
ros2 run my_arm_driver send_joint_target_node \
--ros-args \
-p target_joints_deg:="[20,-30,40,0,0,0]"
```

Expected:

* Driver receives one trajectory point.
* Servos move to the target position.
* `/joint_states` is updated.

Driver output:

```text
Executing trajectory with 1 points
t=2.00s | Servo angles [deg]: [...]
Trajectory execution finished
```

---

# Test 3: Multi-Point Trajectory

Launch the driver:

```bash
ros2 launch my_arm_driver serial_trajectory_bridge.launch.py
```

Run:

```bash
ros2 run my_arm_driver send_joint_trajectory_node
```

Expected:

* Several trajectory points are received.
* The arm moves smoothly through intermediate positions.
* `/joint_states` follows the trajectory.

Driver output:

```text
Executing trajectory with 5 points
t=1.00s ...
t=2.00s ...
t=3.00s ...
Trajectory execution finished
```

---

# Test 4: Verify Joint Trajectory Topic

Open:

```bash
ros2 topic echo /arm_controller/joint_trajectory
```

Send a target:

```bash
ros2 run my_arm_driver send_joint_target_node
```

Expected:

```yaml
joint_names:
- arm_joint1
- arm_joint2
- arm_joint3
- arm_joint4
- arm_joint5
- arm_joint6

points:
- positions: [...]
```

---

# Test 5: Verify Joint States

Open:

```bash
ros2 topic echo /joint_states
```

Move the arm:

```bash
ros2 run my_arm_driver send_joint_trajectory_node
```

Expected:

```yaml
name:
- arm_joint1
- arm_joint2
- arm_joint3
- arm_joint4
- arm_joint5
- arm_joint6

position:
- ...
```

The positions should change while the trajectory is being executed.

---
