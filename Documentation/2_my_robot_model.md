# Robot Models and Gazebo Simulation

This repository contains several robot-arm models described with **URDF** and **Xacro**:

- generic 6-DoF robot arm;
- PUMA robot arm;
- UR5e robot arm;
- custom 5-DoF arm mounted on a mecanum platform.

<p align="center">
  <img src="./Images/Puma.png" alt="Puma Robot" width="200">
  <img src="./Images/UR5e.png" alt="UR5e Robot" width="250">
  <img src="./Images/my_arm_mecanum_5dof.jpg" alt="Mecanum 5DoF Arm" width="250">
</p>

The robot description defines the links, joints, joint axes, reference frames, visual geometry, collision geometry and tool frame.

## Visualize a model in RViz2

RViz2 displays the robot model and its TF frames, but it does not simulate physics.

### Generic 6-DoF arm

```bash
ros2 launch my_arm_description display.launch.py \
  use_sim_time:=false \
  model:=my_arm.urdf.xacro
```
![](./Images/my_arm_rviz.png)

### PUMA arm

```bash
ros2 launch my_arm_description display.launch.py \
  use_sim_time:=false \
  model:=my_arm_puma.urdf.xacro
```
![](./Images/my_arm_puma_rviz.png)

### UR5e arm

```bash
ros2 launch my_arm_description display.launch.py \
  use_sim_time:=false \
  model:=my_arm_ur5e.urdf.xacro
```
![](./Images/my_arm_ur5e_rviz.png)

### Mecanum 5-DoF arm

```bash
ros2 launch my_arm_description display.launch.py \
  use_sim_time:=false \
  model:=my_arm_mecanum_5dof.urdf.xacro
```
![](./Images/my_arm_mecanum_5dof_rviz.png)

## RViz2 and Gazebo

RViz2 is mainly a visualization tool. Gazebo is a physics simulator and can simulate gravity, collisions, joint dynamics, actuators, sensors and ROS 2 controllers.

A model can be displayed in RViz2 without controllers. To move the robot in Gazebo, the model must include a `ros2_control` interface and suitable controllers.

## ros2_control architecture

```text
Joint trajectory
       ↓
arm_controller
       ↓
controller_manager
       ↓
Gazebo ros2_control plugin
       ↓
Simulated robot joints
```

The main controllers used in this repository are:

- `joint_state_broadcaster`: publishes the current joint states;
- `arm_controller`: receives and executes joint trajectories.

## Bring up the robot in Gazebo

Example with the PUMA model:

```bash
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py \
  use_sim_time:=true \
  model:=my_arm_puma.urdf.xacro
```
![](./Images/my_arm_puma2_gz.png)

Example with the UR5e model:

```bash
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py \
  use_sim_time:=true \
  model:=my_arm_ur5e.urdf.xacro
```
![](./Images/my_arm_ur5e_gz.png)

The bringup normally starts Gazebo, loads the selected Xacro model, spawns the robot and activates the controllers.

Check the controller state:

```bash
ros2 control list_controllers
```

Expected active controllers:

```text
joint_state_broadcaster
arm_controller
```

Check the current joint values:

```bash
ros2 topic echo /joint_states --once
```

## Send a joint-space trajectory

This first motion example sends desired joint values directly. It does not use inverse kinematics.

```bash
ros2 launch my_arm_control send_joint_trajectory.launch.py \
  use_sim_time:=true
```

The input is a joint configuration:

```text
q = [q1, q2, q3, q4, q5, q6]
```
![](./Images/send_joints_puma.png)
![](./Images/send_joints_ur5e.png)

In the next document, a Cartesian TCP pose is converted into these joint values using robot kinematics.
