# Arm Motion with MoveIt 2

MoveIt 2 provides a general framework for robot-arm motion.

It combines:

- robot description from URDF;
- semantic information from SRDF;
- inverse kinematics;
- collision checking;
- motion planning;
- trajectory execution through `ros2_control`.

## Motion-planning flow

```text
Cartesian target
       ↓
MoveIt inverse kinematics
       ↓
Valid joint configuration
       ↓
Motion planner
       ↓
Collision-free trajectory
       ↓
FollowJointTrajectory
       ↓
arm_controller
       ↓
Gazebo robot
```

The analytical PUMA example computes one IK solution directly. MoveIt adds trajectory planning, collision checking and a general interface for different robots.

## Start the simulation

### PUMA

```bash
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py \
  use_sim_time:=true \
  model:=my_arm_puma.urdf.xacro
```

### UR5e

```bash
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py \
  use_sim_time:=true \
  model:=my_arm_ur5e.urdf.xacro
```

## Start MoveIt 2

### PUMA

```bash
ros2 launch puma_moveit_config move_group.launch.py \
  use_sim_time:=true
```

### UR5e with RViz2

```bash
ros2 launch ur5e_moveit_config move_group_rviz.launch.py \
  use_sim_time:=true
```

## Move to one Cartesian pose

Example for the PUMA arm:

```bash
ros2 launch my_arm_motion arm_pose.launch.py \
  use_sim_time:=true \
  target_xyz:="[140,-800,300]" \
  target_rpy:="[0.0,70.0,-90.0]" \
  seed_from_joint_states:=false \
  seed_joints:="[-90,-40,30,0,0,0]" \
  execute:=true
```

Example for the UR5e arm:

```bash
ros2 launch my_arm_motion arm_pose.launch.py \
  use_sim_time:=true \
  target_xyz:="[0,-400,500]" \
  target_rpy:="[0.0,0.0,0.0]" \
  seed_from_joint_states:=false \
  seed_joints:="[-60,-60,-100,-90,-90,0]" \
  execute:=true
```

The seed joint values provide an initial configuration for the IK solver. Different seeds may produce different valid robot configurations.

## Execute a pose sequence

PUMA example:

```bash
ros2 launch my_arm_motion arm_pose_sequence.launch.py \
  use_sim_time:=true \
  sequence_file:=puma_handshake.yaml
```

UR5e example:

```bash
ros2 launch my_arm_motion arm_pose_sequence.launch.py \
  use_sim_time:=true \
  sequence_file:=ur5e_handshake.yaml
```

A sequence file defines several Cartesian targets that are planned and executed in order.

## Analytical IK and MoveIt 2

| Analytical PUMA IK | MoveIt 2 |
|---|---|
| Robot-specific equations | General planning framework |
| Direct joint solution | IK plus trajectory planning |
| Useful to study robot geometry | Useful for complete robot applications |
| No collision planning | Collision checking available |
| Explicit elbow and wrist branches | Solution depends on solver and seed |

The two approaches are complementary: analytical IK helps to understand the robot, while MoveIt 2 provides the tools required for complete motion planning.
