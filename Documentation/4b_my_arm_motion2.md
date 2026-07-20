# Arm motion with MoveIt 2

In the previous document, we created and configured a MoveIt 2 package for each robot arm.

MoveIt now knows:

- the robot model from the URDF;
- the semantic model from the SRDF;
- the numerical inverse kinematics solver;
- the joint limits;
- the allowed self-collisions;
- the motion planner;
- the trajectory controller.

In this document, we use the `my_arm_motion` package to plan and execute two basic industrial robot movements:

- **MoveJ:** joint-space motion to a Cartesian target;
- **MoveL:** straight-line Cartesian motion to a Cartesian target.

We also use optional trajectory checks to detect configurations close to singularities.

This document only covers single MoveJ and MoveL commands. Motion sequences, Planning Scene objects and automatic selection between different planning candidates will be studied later.

## 1. Why use numerical inverse kinematics?

Inverse kinematics calculates the joint angles required to reach a desired end-effector pose.

Analytical inverse kinematics uses equations developed for one specific robot geometry. It can be fast and can show the different elbow and wrist solutions explicitly. However:

- the equations are different for every robot;
- they can be difficult to derive;
- not every robot has a simple analytical solution;
- modifications to the robot geometry may require new equations.

MoveIt normally uses a numerical IK solver. The solver searches for a valid joint configuration starting from an initial configuration called the **IK seed**.

Numerical IK is useful here because the same motion nodes can be used with different robot arms. Only the MoveIt configuration package and robot model need to be adapted.

## 2. What MoveIt adds

Inverse kinematics only finds a joint configuration for a target pose. It does not describe how the robot should move to that configuration.

MoveIt adds:

- joint-limit checking;
- self-collision checking;
- collision checking with Planning Scene objects;
- joint-space motion planning with OMPL;
- Cartesian-path calculation;
- velocity and acceleration scaling;
- trajectory execution through `ros2_control`.

A valid IK solution does not guarantee that a valid trajectory exists. For this reason, the nodes separate three operations:

1. calculate or follow the inverse kinematics;
2. plan and validate the trajectory;
3. execute the trajectory.

## 3. The motion nodes

The current examples use four nodes:

| Python node | Launch file | Motion | Singularity check |
|---|---|---|---|
| `arm_movej.py` | `arm_movej.launch.py` | Joint-space MoveJ | No |
| `arm_movel.py` | `arm_movel.launch.py` | Straight Cartesian MoveL | No |
| `arm_movej_sing.py` | `arm_movej_sing.launch.py` | Joint-space MoveJ | Yes |
| `arm_movel_sing.py` | `arm_movel_sing.launch.py` | Straight Cartesian MoveL | Yes |

The package structure used in this document is:

```text
my_arm_motion/
├── launch/
│   ├── arm_movej.launch.py
│   ├── arm_movel.launch.py
│   ├── arm_movej_sing.launch.py
│   └── arm_movel_sing.launch.py
└── my_arm_motion/
    ├── arm_movej.py
    ├── arm_movel.py
    ├── arm_movej_sing.py
    └── arm_movel_sing.py
```

The examples assume that all robot arms use:

- planning group: `arm`;
- base frame: `base_link`;
- end-effector link: `tool`;
- joints: `joint1` to `joint6`.

## 4. Start the simulation

Open one terminal and launch the robot in Gazebo.

### PUMA

```bash
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py \
  use_sim_time:=true \
  model:=my_arm_puma.urdf.xacro \
  controllers:=gz_controllers.yaml \
  use_gripper:=false
```

### UR5e

```bash
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py \
  use_sim_time:=true \
  model:=my_arm_ur5e.urdf.xacro \
  controllers:=gz_controllers.yaml \
  use_gripper:=false
```

Check the controllers and joint states:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

## 5. Start MoveIt and RViz2

Open a second terminal and start `move_group`.

### PUMA

```bash
ros2 launch puma_moveit_config move_group.launch.py \
  use_sim_time:=true
```

### UR5e

```bash
ros2 launch ur5e_moveit_config move_group.launch.py \
  use_sim_time:=true
```

Open a third terminal and start RViz2.

### PUMA

```bash
ros2 launch puma_moveit_config moveit_rviz.launch.py \
  use_sim_time:=true
```

### UR5e

```bash
ros2 launch ur5e_moveit_config moveit_rviz.launch.py \
  use_sim_time:=true
```

In these packages, `moveit_rviz.launch.py` starts RViz2 but does not replace `move_group.launch.py`. Both terminals must remain active.

RViz2 is not required for execution, but it is useful for observing the robot state, the planned trajectory, self-collisions and Planning Scene objects.

Useful MoveIt interfaces are:

```bash
ros2 service list | grep compute_ik
ros2 service list | grep compute_fk
ros2 service list | grep compute_cartesian_path
ros2 action list | grep execute_trajectory
```

## 6. Cartesian targets

All four launch files use:

- `target_xyz`: position `[x, y, z]` in millimetres;
- `target_rpy`: orientation `[roll, pitch, yaw]` in degrees.

The launch file converts millimetres to metres and degrees to radians. The Python node converts roll, pitch and yaw to a quaternion.

The target is normally defined in `base_link` and must be reached by the `tool` link.

## 7. MoveJ: joint-space motion

MoveJ receives a Cartesian target, but the planned path is in joint space.

The node performs this flow:

```text
Cartesian target
      ↓
Numerical IK with a seed
      ↓
Collision-aware joint goal
      ↓
OMPL joint-space planning
      ↓
Collision-checked joint trajectory
      ↓
ExecuteTrajectory
```

The end effector does not normally follow a straight line during MoveJ. Each joint follows the planned joint trajectory.

MoveJ is normally used for:

- large movements in free space;
- moving to an approach configuration;
- returning to a safe position;
- avoiding obstacles known by MoveIt.

### 7.1 Plan a UR5e MoveJ

This example plans a movement to an approach pose:

```bash
ros2 launch my_arm_motion arm_movej.launch.py \
  use_sim_time:=true \
  target_xyz:="[0,-400,450]" \
  target_rpy:="[90.0,0.0,0.0]" \
  seed_from_joint_states:=true \
  max_velocity:=0.1 \
  max_acceleration:=0.1 \
  execute:=false
```

`execute:=false` still calculates IK and plans the complete MoveJ trajectory. It only disables trajectory execution.

If planning succeeds, execute the movement:

```bash
ros2 launch my_arm_motion arm_movej.launch.py \
  use_sim_time:=true \
  target_xyz:="[0,-400,450]" \
  target_rpy:="[90.0,0.0,0.0]" \
  seed_from_joint_states:=true \
  max_velocity:=0.1 \
  max_acceleration:=0.1 \
  execute:=true
```

### 7.2 The IK seed

With:

```bash
seed_from_joint_states:=true
```

the current joint state is used as the IK seed. This normally selects a nearby IK branch and reduces unnecessary configuration changes.

A manual seed can also be used:

```bash
seed_from_joint_states:=false \
seed_joints:="[-60,-60,-100,170,-90,0]"
```

Different seeds can produce different elbow or wrist configurations for the same Cartesian target. A different IK branch can also produce a very different planned trajectory.

## 8. MoveL: straight Cartesian motion

MoveL calculates a Cartesian path from the current end-effector pose to the target pose.

```text
Current tool pose
      ↓
Cartesian interpolation to the target
      ↓
IK along the Cartesian waypoints
      ↓
Collision and joint-jump checking
      ↓
Time-parameterized joint trajectory
      ↓
ExecuteTrajectory
```

The tool position follows a straight Cartesian line. If the orientation changes, it is interpolated along the path.

MoveL is normally used for:

- approaching or leaving an object;
- insertion and extraction;
- welding or dispensing;
- moving through a work area along a controlled line.

MoveL is not an OMPL path around obstacles. If the requested line crosses an obstacle, joint limit or unreachable region, the Cartesian path should be rejected or incomplete.

### 8.1 Move 50 mm down

First place the UR5e at the approach pose `[0,-400,450]` with the MoveJ example.

Then plan a 50 mm vertical MoveL:

```bash
ros2 launch my_arm_motion arm_movel.launch.py \
  use_sim_time:=true \
  target_xyz:="[0,-400,400]" \
  target_rpy:="[90.0,0.0,0.0]" \
  max_step:=0.005 \
  fraction_threshold:=1.0 \
  avoid_collisions:=true \
  execute:=false
```

The most important result is:

```text
Cartesian path completed fraction: 1.000 (100.0%)
```

`fraction_threshold:=1.0` requires the complete Cartesian path. A partial path is not accepted as a successful MoveL.

If planning succeeds, execute it:

```bash
ros2 launch my_arm_motion arm_movel.launch.py \
  use_sim_time:=true \
  target_xyz:="[0,-400,400]" \
  target_rpy:="[90.0,0.0,0.0]" \
  max_step:=0.005 \
  fraction_threshold:=1.0 \
  avoid_collisions:=true \
  max_velocity:=0.1 \
  max_acceleration:=0.1 \
  execute:=true
```

## 9. MoveJ and MoveL comparison

| Property | MoveJ | MoveL |
|---|---|---|
| Target input | Cartesian pose | Cartesian pose |
| Main planning space | Joint space | Cartesian space |
| IK use | Finds the final joint goal | Solves the interpolated Cartesian waypoints |
| Tool path | Not necessarily straight | Straight line |
| Obstacle behaviour | OMPL may find another joint path | Cannot move around an obstacle and remain a straight MoveL |
| Typical use | Free-space and approach motions | Controlled motion near the task |

The two commands are complementary. A common industrial pattern is:

```text
MoveJ to an approach pose
MoveL to the work pose
MoveL back to the approach pose
```

## 10. Collision checking

The nodes can check:

- robot self-collisions;
- joint limits;
- collisions with objects in the MoveIt Planning Scene.

For MoveJ, MoveIt first requests collision-aware IK and then plans a collision-checked OMPL trajectory.

For MoveL, collision checking is applied while the Cartesian path is calculated when:

```bash
avoid_collisions:=true
```

MoveIt only knows the world geometry included in its Planning Scene. An object visible in Gazebo is not automatically a collision object in MoveIt.

Later, we will add objects such as a table or box to the Planning Scene and compare the behaviour of MoveJ and MoveL.

## 11. Why check singularities?

A singularity is a robot configuration where one or more Cartesian motion directions are lost or where a small Cartesian velocity can require very large joint velocities.

Near a singularity:

- numerical IK can become unstable;
- small pose changes can produce large joint changes;
- the wrist or elbow configuration can change unexpectedly;
- a Cartesian MoveL can become impractical;
- trajectory tracking can become more difficult.

Collision-free planning does not automatically guarantee that the trajectory stays far from singularities.

## 12. Numerical singularity check

The `_sing` nodes add a safety check after planning and before execution.

For selected trajectory points, the node:

1. calls MoveIt forward kinematics;
2. perturbs each joint by a small value;
3. calculates a numerical geometric Jacobian;
4. computes its singular values with SVD;
5. checks the minimum singular value;
6. checks the Jacobian condition number;
7. checks the maximum joint jump between consecutive trajectory points.

The trajectory is rejected when:

```text
sigma_min < min_singular_value
```

or:

```text
condition_number > max_condition_number
```

A larger minimum singular value and a smaller condition number normally indicate a configuration farther from a singularity.

These values are practical numerical indicators, not an absolute mathematical safety guarantee. The geometric Jacobian combines translational and rotational components, so the thresholds must be validated for each robot model.

## 13. MoveJ with singularity checking

Plan and check the complete MoveJ trajectory:

```bash
ros2 launch my_arm_motion arm_movej_sing.launch.py \
  use_sim_time:=true \
  target_xyz:="[0,-400,450]" \
  target_rpy:="[90.0,0.0,0.0]" \
  seed_from_joint_states:=true \
  singularity_samples:=0 \
  min_singular_value:=0.01 \
  max_condition_number:=200.0 \
  max_joint_jump_deg:=45.0 \
  execute:=false
```

With `singularity_samples:=0`, every trajectory point is checked. This is the most complete test, but it can take a long time for a large MoveJ trajectory.

A successful result has this form:

```text
MoveJ planning succeeded: ... trajectory points.
Maximum planned joint jump: ... deg.
Singularity check passed.
```

Execute only after the check succeeds:

```bash
ros2 launch my_arm_motion arm_movej_sing.launch.py \
  use_sim_time:=true \
  target_xyz:="[0,-400,450]" \
  target_rpy:="[90.0,0.0,0.0]" \
  seed_from_joint_states:=true \
  singularity_samples:=0 \
  min_singular_value:=0.01 \
  max_condition_number:=200.0 \
  max_joint_jump_deg:=45.0 \
  max_velocity:=0.1 \
  max_acceleration:=0.1 \
  execute:=true
```

If a trajectory is rejected, the current node does not automatically calculate another one. Try another IK seed, another target, another approach pose or another robot configuration.

## 14. MoveL with singularity checking

Starting from `[0,-400,450]`, check the complete 50 mm MoveL:

```bash
ros2 launch my_arm_motion arm_movel_sing.launch.py \
  use_sim_time:=true \
  target_xyz:="[0,-400,400]" \
  target_rpy:="[90.0,0.0,0.0]" \
  max_step:=0.005 \
  fraction_threshold:=1.0 \
  avoid_collisions:=true \
  singularity_samples:=0 \
  min_singular_value:=0.01 \
  max_condition_number:=200.0 \
  max_joint_jump_deg:=45.0 \
  execute:=false
```

If the complete Cartesian path and all safety checks succeed, execute it:

```bash
ros2 launch my_arm_motion arm_movel_sing.launch.py \
  use_sim_time:=true \
  target_xyz:="[0,-400,400]" \
  target_rpy:="[90.0,0.0,0.0]" \
  max_step:=0.005 \
  fraction_threshold:=1.0 \
  avoid_collisions:=true \
  singularity_samples:=0 \
  min_singular_value:=0.01 \
  max_condition_number:=200.0 \
  max_joint_jump_deg:=45.0 \
  max_velocity:=0.1 \
  max_acceleration:=0.1 \
  execute:=true
```

If a strict MoveL crosses a singularity, the correct result is to reject it. Moving around the singularity would no longer produce the requested straight line. Possible solutions are to change the approach configuration, tool orientation or Cartesian target.

## 15. Main parameters

### Common parameters

| Parameter | Meaning | Typical value |
|---|---|---|
| `target_xyz` | Target position in millimetres | `[0,-400,450]` |
| `target_rpy` | Target RPY orientation in degrees | `[90,0,0]` |
| `avoid_collisions` | Enable collision checking | `true` |
| `max_velocity` | Velocity scaling factor | `0.1` or `0.2` |
| `max_acceleration` | Acceleration scaling factor | `0.1` or `0.2` |
| `motion_timeout_sec` | Planning or execution wait timeout | `180.0` |
| `execute` | Execute the validated trajectory | `false` first |
| `use_sim_time` | Use the Gazebo clock | `true` |

### MoveJ parameters

| Parameter | Meaning |
|---|---|
| `seed_from_joint_states` | Use the current configuration as the IK seed |
| `seed_joints` | Manual IK seed in degrees |
| `ik_timeout_sec` | Numerical IK timeout |
| `joint_tolerance` | Joint-goal planning tolerance |

### MoveL parameters

| Parameter | Meaning |
|---|---|
| `max_step` | Maximum Cartesian interpolation step in metres |
| `fraction_threshold` | Minimum accepted fraction of the Cartesian path |
| `jump_threshold` | MoveIt relative joint-jump threshold; `0.0` disables this internal check |

The `_sing` nodes also apply their independent absolute check using `max_joint_jump_deg`, even when `jump_threshold:=0.0`.

### Singularity-check parameters

| Parameter | Meaning |
|---|---|
| `check_singularities` | Enable or disable the Jacobian check |
| `jacobian_delta` | Small joint perturbation used for the numerical Jacobian |
| `singularity_samples` | Number of checked points; `0` checks every point |
| `min_singular_value` | Minimum accepted smallest singular value |
| `max_condition_number` | Maximum accepted Jacobian condition number |
| `max_joint_jump_deg` | Maximum accepted joint change between consecutive points |

Using 20 samples is faster for initial experiments. Checking every point is slower but is recommended for final validation because sparse sampling can miss a critical point.

## 16. Common problems

### `/compute_ik`, `/compute_fk` or planning interfaces are unavailable

The MoveIt `move_group` node is probably not running.

### No `/joint_states`

Check Gazebo, the joint-state broadcaster and the joint names:

```bash
ros2 topic echo /joint_states --once
```

### IK fails

Possible causes are:

- unreachable position;
- impossible orientation;
- unsuitable IK seed;
- target close to a singularity;
- incorrect group or tool link.

Try a closer target, another orientation or another seed.

### MoveL fraction is below 1.0

The complete straight path could not be calculated. Possible causes include collisions, joint limits, unreachable waypoints, singularities or an unsuitable starting configuration.

Do not execute a partial path when a complete MoveL is required.

### Singularity check rejects the trajectory

The path contains at least one sampled point below the singular-value limit or above the condition-number limit.

For MoveJ, try another seed, target or intermediate approach pose. For MoveL, change the start configuration, orientation or line.

### Execution timeout

Low velocity scaling and long trajectories can require more time. Increase `motion_timeout_sec` only after confirming that the robot is moving correctly and the controller is active.

## 17. Recommended workflow

For every new movement:

1. start Gazebo, `move_group` and RViz2;
2. use low velocity and acceleration values;
3. run the selected node with `execute:=false`;
4. verify that planning succeeds;
5. for MoveL, require `fraction=1.0`;
6. inspect the collision and singularity results;
7. execute only in simulation first;
8. stop if the robot changes IK branch unexpectedly;
9. validate the Planning Scene before using obstacles;
10. perform additional safety validation before using a real robot.

## 18. Next steps

Later documents can add:

- YAML sequences that combine MoveJ and MoveL;
- collision objects in the Planning Scene;
- alternative IK and OMPL planning candidates;
- automatic selection of the safest trajectory;
- gripper commands;
- validation with a real robot.

## 19. Main conclusions

- Numerical IK provides a general solution for different robot arms.
- The IK seed can select a different elbow or wrist configuration.
- MoveJ plans in joint space and does not produce a straight tool path.
- MoveL follows a straight Cartesian path and cannot plan around an obstacle.
- MoveIt checks only obstacles included in its Planning Scene.
- Collision-free planning does not automatically guarantee singularity avoidance.
- The `_sing` nodes validate the planned trajectory before execution.
- `execute:=false` should always be used before executing a new movement.

````bash
ros2 launch my_arm_motion arm_motion_sequence.launch.py \
  use_sim_time:=true \
  sequence_file:=ur5e_handshake_plan.yaml \
  avoid_collisions:=true \
  check_singularities:=true \
  execute:=false
````
```bash
ros2 launch my_arm_motion arm_motion_sequence_saved.launch.py \
  use_sim_time:=true \
  sequence_file:=ur5e_handshake_plan.yaml \
  trajectory_filename:=ur5e_handshake_planned.yaml \
  avoid_collisions:=true \
  check_singularities:=true \
  min_singular_value:=0.008 \
  max_condition_number:=250.0 \
  save_trajectory:=true \
  execute:=false
```
```bash
ros2 launch my_arm_motion arm_execute_saved.launch.py \
  use_sim_time:=true \
  trajectory_filename:=ur5e_handshake_planned.yaml \
  start_tolerance_deg:=5.0 \
  execute:=true
```
