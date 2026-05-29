# PUMA Robot Arm - FK and IK Examples

## Overview

This tutorial demonstrates how to:

1. Launch the PUMA robot simulation.
2. Execute Forward Kinematics (FK) using joint angles.
3. Execute Inverse Kinematics (IK) for position control.
4. Execute Inverse Kinematics (IK) for full pose control (position + orientation).

The examples use:

* ROS 2 Humble
* Gazebo
* RViz
* ros2_control
* JointTrajectory controller

---

# 1. Build the Workspace

```bash
cd ~/my_rUBot_arm

colcon build --symlink-install

source install/setup.bash
```

---

# 2. Launch the PUMA Simulation

Start Gazebo, RViz and ros2_control:

```bash
ros2 launch my_arm_gz bringup_puma.launch.py
```

Verify that the controller is active:

```bash
ros2 control list_controllers
```

Expected output:

```text
joint_state_broadcaster   active
arm_controller            active
```

---

# 3. Forward Kinematics (FK)

The FK node moves the robot to a specified joint configuration and then reads the TCP pose from TF.

## Execute FK

```bash
ros2 run my_arm_kinematics fkine_puma_exe --ros-args \
  -p controller_topic:=/arm_controller/joint_trajectory
```

Default joint target:

```text
q1 = 0°
q2 = -30°
q3 = 60°
q4 = 0°
q5 = 45°
q6 = 0°
```

Example output:

```text
Position [m]:
x=0.5422
y=0.1397
z=0.9423

Orientation RPY [deg]:
roll=-0.00
pitch=43.97
yaw=0.00
```

## Custom Joint Configuration

```bash
ros2 run my_arm_kinematics fkine_puma_exe --ros-args \
  -p controller_topic:=/arm_controller/joint_trajectory \
  -p target_deg:="[45.0,-30.0,60.0,0.0,45.0,0.0]"
```

---

# 4. Inverse Kinematics - Position Only

This example computes:

```text
XYZ → q1,q2,q3
```

while keeping:

```text
q4 = q5 = q6 = 0
```

The objective is to understand the geometric IK solution of the first three joints.

## Execute Position IK

```bash
ros2 run my_arm_kinematics ikine_puma_position_exe --ros-args \
  -p controller_topic:=/arm_controller/joint_trajectory \
  -p target_xyz:="[0.35,0.14,0.55]"
```

Example output:

```text
Target XYZ [m]:
[0.35, 0.14, 0.55]

IK solution [deg]:
[0.0, -25.4, 61.8, 0.0, 0.0, 0.0]
```

---

# 5. Inverse Kinematics - Full Pose

This example computes:

```text
XYZ + RPY → q1,q2,q3,q4,q5,q6
```

The first three joints position the wrist.

The wrist orientation is computed using:

```text
R36 = R03ᵀ · R06
```

which correctly accounts for the orientation already introduced by joints 1, 2 and 3.

---

## Execute Pose IK

```bash
ros2 run my_arm_kinematics ikine_puma_pose_exe --ros-args \
  -p controller_topic:=/arm_controller/joint_trajectory \
  -p target_xyz:="[0.35,0.14,0.55]" \
  -p target_rpy_deg:="[0.0,0.0,0.0]" \
  -p elbow:=up \
  -p wrist:=noflip
```

---

## Example with TCP Pitch

```bash
ros2 run my_arm_kinematics ikine_puma_pose_exe --ros-args \
  -p controller_topic:=/arm_controller/joint_trajectory \
  -p target_xyz:="[0.40,0.14,0.60]" \
  -p target_rpy_deg:="[0.0,20.0,0.0]" \
  -p elbow:=up \
  -p wrist:=noflip
```

---

# 6. Elbow Configurations

Two geometric solutions are available:

## Elbow Up

```bash
-p elbow:=up
```

## Elbow Down

```bash
-p elbow:=down
```

---

# 7. Wrist Configurations

Two wrist solutions are available:

## Non-flip Wrist

```bash
-p wrist:=noflip
```

## Flip Wrist

```bash
-p wrist:=flip
```

---

# 8. TF Verification

The FK and IK nodes automatically verify the TCP pose using TF.

Frames used:

```text
base_link
└── link6
```

The resulting pose is displayed as:

```text
Position [m]
Orientation RPY [deg]
Quaternion
```

---

# Educational Sequence

Recommended order for students:

1. Forward Kinematics

   * Joint angles → TCP pose

2. Position IK

   * TCP position → Joint angles

3. Full Pose IK

   * TCP position + orientation → Joint angles

4. MoveIt 2

   * Numerical IK
   * Motion planning
   * Collision avoidance

5. Real Robot

   * ros2_control
   * Industrial robot controllers

This progression clearly illustrates the difference between analytical kinematics and modern motion-planning frameworks such as MoveIt 2.
