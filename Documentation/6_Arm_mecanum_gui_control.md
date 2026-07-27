# Joint Slider GUI

## Overview

This package provides two Tkinter-based GUI nodes for the rUBot mecanum servo arm:

- **Joint Slider GUI**: directly commands the arm joints and gripper servo.
- **Cartesian Slider GUI**: commands a Cartesian wrist-centre target using analytical inverse kinematics.

Both GUIs are **motion-command nodes**. They do not communicate directly with the Arduino or the servos. Instead, they publish ROS 2 trajectory commands that are executed by the custom hardware driver.

## Control Architecture

```text
Joint Slider GUI
        ↓
JointTrajectory message
        ↓
/arm_controller/joint_trajectory
        ↓
serial_trajectory_bridge_node
        ↓
Serial communication
        ↓
Arduino firmware
        ↓
SG90 servos
```

The GUI publishes desired joint targets.

The `serial_trajectory_bridge_node`:

- receives the trajectory;
- converts ROS joint positions to calibrated servo angles;
- applies servo limits;
- sends the serial command to the Arduino;
- publishes `/joint_states`.

The published `/joint_states` message contains the **last commanded joint positions**, not measured servo positions.

---

# Joint Slider GUI

This node provides a small Tkinter interface for commanding the five arm joints and the gripper servo.

It publishes:

```text
Topic:
/arm_controller/joint_trajectory

Message:
trajectory_msgs/msg/JointTrajectory
```

Each message contains the latest complete joint target.

The publication rate is limited to a maximum of **10 Hz** while a slider is moving. Old intermediate targets are discarded.

If Tkinter is not installed:

```bash
sudo apt install python3-tk
```

The node is installed in the `my_arm_driver` package.

## Build

```bash
cd ~/my_rUBot_arm
colcon build --packages-select my_arm_driver --symlink-install
source install/setup.bash
```

## Run

Terminal 1:

```bash
ros2 launch my_arm_driver serial_trajectory_bridge.launch.py \
    serial_port:=/dev/ttyUSB0
```

Terminal 2:

```bash
ros2 launch my_arm_driver joint_slider_gui.launch.py
```

or

```bash
ros2 run my_arm_driver joint_slider_gui_node
```

The GUI targets are expressed as joint angles in degrees. They are converted into radians inside the published `JointTrajectory`. The serial bridge converts them back into calibrated servo angles using the configured servo centres, directions and limits.

## Optional Parameters

Change the publication rate:

```bash
ros2 launch my_arm_driver joint_slider_gui.launch.py publish_rate_hz:=5.0
```

Change the joint limits:

```bash
ros2 run my_arm_driver joint_slider_gui_node --ros-args \
    -p joint_min_deg:="[-90,-60,-90,-90,-90,-45]" \
    -p joint_max_deg:="[90,60,90,90,90,45]"
```

For the first hardware test, keep the arm clear of obstacles and move one slider at a time through a small angle.

---

# Mecanum Cartesian Slider

This GUI defines a Cartesian wrist-centre target and calculates the first three arm joints using analytical inverse kinematics.

The resulting complete joint target is published as a `trajectory_msgs/msg/JointTrajectory` message and executed by the same custom hardware driver used by the Joint Slider GUI.

## Controls

| GUI control | Robot command |
|---|---|
| `x`, `y`, `z` | Wrist-centre position relative to `base_link` (mm) |
| `joint4` | Wrist pitch (deg) |
| `joint5` | Wrist roll (deg) |
| `Open/Closed` | Calibrated gripper-servo command |
| `Elbow up/down` | Analytical IK solution branch |

The target is published to:

```text
Topic:
/arm_controller/joint_trajectory

Message:
trajectory_msgs/msg/JointTrajectory
```

The node does not publish when it starts.

With **Live send** enabled, it publishes the newest valid target at a maximum rate of 10 Hz while a control is changing.

If Tkinter is not installed:

```bash
sudo apt install python3-tk
```

## Build

```bash
cd ~/my_rUBot_arm
colcon build --packages-select my_arm_kinematics --symlink-install
source install/setup.bash
```

## Run

Terminal 1:

```bash
ros2 launch my_arm_driver serial_trajectory_bridge.launch.py \
    serial_port:=/dev/ttyUSB0
```

Terminal 2:

```bash
ros2 launch my_arm_kinematics mecanum_cartesian_slider.launch.py
```

## Gripper Calibration

The current implementation treats the gripper as the sixth joint command.

Configure the open and closed gripper angles:

```bash
ros2 launch my_arm_kinematics mecanum_cartesian_slider.launch.py \
    gripper_open_joint_deg:=0.0 \
    gripper_closed_joint_deg:=45.0
```

The serial bridge converts each joint command into a physical servo angle using the configured servo centre, direction and limits.

## Safety and IK Behaviour

The node verifies:

- the wrist-centre target is reachable;
- the calculated joints are inside their configured limits;
- the target is not too close to the joint1 axis.

Invalid targets are displayed in red and are not published.

For the first hardware test:

- disable **Live send**;
- keep the arm clear of obstacles;
- use the **Send target** button.

## Wrist Centre and TCP

The analytical IK controls the wrist-centre position, not the final TCP position.

Changing `joint4` therefore changes the position of the tool or gripper tip.

A future version can include the final link and tool offset so that the analytical IK directly controls the TCP position.
