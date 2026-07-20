
# Joint slider GUI 

This proposal adds a small Tkinter GUI that commands the six arm joints through:

```text
/arm_controller/joint_trajectory
```

The GUI publishes the latest complete target at a maximum rate of 10 Hz while a slider is moving. It does not start six external ROS processes and it does not publish old intermediate targets.

If Tkinter is not already installed:

```bash
sudo apt install python3-tk
```
This node is installed on the package `my_arm_driver`

## Build

```bash
cd ~/my_rUBot_arm
colcon build --packages-select my_arm_driver --symlink-install
source install/setup.bash
```

## Run

Terminal 1 — start the physical arm driver:

```bash
ros2 launch my_arm_driver serial_trajectory_bridge.launch.py \
    serial_port:=/dev/ttyUSB0
```

Terminal 2 — start the slider GUI:

```bash
ros2 launch my_arm_driver joint_slider_gui.launch.py
```

You can also run the node directly:

```bash
ros2 run my_arm_driver joint_slider_gui_node
```

The default slider range is `-90` to `+90` degrees. These joint targets are converted by the existing bridge to servo commands centered at 90 degrees and limited to the configured servo range.

## Optional parameters

Change the rate:

```bash
ros2 launch my_arm_driver joint_slider_gui.launch.py publish_rate_hz:=5.0
```

Change the limits directly with ROS parameters:

```bash
ros2 run my_arm_driver joint_slider_gui_node --ros-args \
    -p joint_min_deg:="[-90,-60,-90,-90,-90,-45]" \
    -p joint_max_deg:="[90,60,90,90,90,45]"
```

For the first hardware test, keep the arm clear of obstacles and move one slider at a time through a small angle.

# Mecanum Cartesian slider

This proposal adds a GUI to `my_arm_kinematics`. The GUI defines the position of the wrist centre (`joint4`) and uses analytical inverse kinematics to calculate `joint1`, `joint2` and `joint3`.

## Controls

| GUI control | Robot command |
|---|---|
| `x`, `y`, `z` | Wrist-centre position relative to `base_link`, in millimetres |
| `joint4` | Wrist pitch, in degrees |
| `joint5` | Gripper roll, in degrees |
| `Open/Closed` | Calibrated joint6 servo target |
| `Elbow up/down` | Analytical IK solution branch |

The target is published to:

```text
/arm_controller/joint_trajectory
```

The node does not publish when it starts. With `Live send` enabled, it publishes the newest valid target at a maximum rate of 10 Hz while a control is changing.


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

## Run with the physical arm

Terminal 1:

```bash
ros2 launch my_arm_driver serial_trajectory_bridge.launch.py \
    serial_port:=/dev/ttyUSB0
```

Terminal 2:

```bash
ros2 launch my_arm_kinematics mecanum_cartesian_slider.launch.py
```

## Gripper calibration

The current serial bridge treats all six commands as relative servo angles. For this reason, this first version uses two configurable joint6 angles:

```bash
ros2 launch my_arm_kinematics mecanum_cartesian_slider.launch.py \
    gripper_open_joint_deg:=0.0 \
    gripper_closed_joint_deg:=45.0
```

Change these two values after checking the real gripper direction and mechanical limits.

## Safety and IK behaviour

The node verifies:

- whether the wrist target is reachable;
- whether the calculated joints are within their configured limits;
- whether the target is too close to the joint1 axis.

An invalid target is displayed in red and is not published. For the first hardware test, disable `Live send`, keep the arm clear of obstacles, and use the `Send target` button.

The Cartesian target is the position of the wrist centre, not the final TCP. Moving joint4 therefore changes the TCP position. A later version can compensate for `L4 + tool_x` and control the final TCP analytically.