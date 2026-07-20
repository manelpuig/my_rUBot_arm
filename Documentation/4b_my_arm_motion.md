# Arm motion planning and execution with MoveIt 2

This document presents the nodes included in `my_arm_motion` for planning,
validating, saving and executing robot-arm trajectories.

The examples use the UR5e, the MoveIt planning group `arm`, the base frame
`base_link`, the tool link `tool`, and joints `joint1` to `joint6`.
Cartesian positions passed to launch files are expressed in millimetres and
RPY angles in degrees.

Always plan with `execute:=false` first. Execute only after checking the
trajectory in RViz2 and verifying the Planning Scene.

## 1. Nodes overview

| Node | Launch file | Purpose |
|---|---|---|
| `arm_pose_numeric_ik.py` | `arm_pose_numeric_ik.launch.py` | Computes numerical IK and optionally executes the resulting joint target directly. |
| `arm_movej.py` | `arm_movej.launch.py` | Plan a joint-space MoveJ to a Cartesian target. |
| `arm_movel.py` | `arm_movel.launch.py` | Plan a straight Cartesian MoveL. |
| `arm_movej_sing.py` | `arm_movej_sing.launch.py` | MoveJ with numerical Jacobian and joint-jump checks. |
| `arm_movel_sing.py` | `arm_movel_sing.launch.py` | MoveL with numerical Jacobian and joint-jump checks. |
| `arm_movej_candidates.py` | `arm_movej_candidates.launch.py` | Test several IK solutions and OMPL plans, select the best MoveJ and optionally save it. |
| `arm_movel_candidates.py` | `arm_movel_candidates.launch.py` | Test different Cartesian interpolation resolutions, select the best MoveL and optionally save it. |
| `arm_test_scene.py` | `arm_test_scene.launch.py` | Add or remove a box in the MoveIt Planning Scene. |
| `arm_pose_sequence.py` | `arm_pose_sequence.launch.py` | Execute a YAML pose sequence directly through `FollowJointTrajectory`, without path planning. |
| `arm_motion_sequence.py` | `arm_motion_sequence.launch.py` | Plan, validate and optionally execute a YAML sequence of MoveJ and MoveL segments. |
| `arm_motion_sequence_saved.py` | `arm_motion_sequence_saved.launch.py` | Plan, validate, retime, concatenate and save a complete sequence. |
| `arm_execute_saved.py` | `arm_execute_saved.launch.py` | Validate the start state and execute a saved YAML trajectory without replanning. |

The candidate nodes reuse the singularity-check implementations through
inheritance:

```text
ArmMoveJ -> ArmMoveJSingularityChecked -> ArmMoveJCandidates
ArmMoveL -> ArmMoveLSingularityChecked -> ArmMoveLCandidates
```

This does not start several ROS nodes. Launching a candidate executable starts
one node containing the inherited functionality.

## Start the system

Build and source the workspace after modifying a Python, launch or YAML file:

```bash
cd ~/my_rUBot_arm
colcon build --packages-select my_arm_motion --symlink-install
source install/setup.bash
```

Start the UR5e simulation:

```bash
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py \
  use_sim_time:=true \
  model:=my_arm_ur5e.urdf.xacro \
  controllers:=gz_controllers.yaml \
  use_gripper:=false
```

Start MoveIt and RViz2 in two additional terminals:

```bash
ros2 launch ur5e_moveit_config move_group.launch.py use_sim_time:=true
```

```bash
ros2 launch ur5e_moveit_config moveit_rviz.launch.py use_sim_time:=true
```

Check the main interfaces:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 service list | grep compute_ik
ros2 action list | grep execute_trajectory
```

<!-- Screenshot: Gazebo and RViz2 with the UR5e ready. -->

## 2. Basic cartesian POSE example
### `arm_pose_numeric_ik.py`: single-pose numerical IK

This node is the simplest Cartesian-pose example in the package. It sends one target pose to MoveIt's `/compute_ik` service and obtains the corresponding values for `joint1` to `joint6`. It does not use OMPL, collision checking or singularity analysis.

With `execute:=false`, the node only calculates and prints the numerical IK solution:

```bash
ros2 launch my_arm_motion arm_pose_numeric_ik.launch.py \
  use_sim_time:=true \
  target_xyz:="[0,-400,500]" \
  target_rpy:="[90,0,0]" \
  seed_from_joint_states:=false \
  seed_joints:="[-60,-60,-100,170,-90,0]" \
  ik_timeout_sec:=1.0 \
  execute:=false
```

With `execute:=true`, the resulting joint configuration is sent directly to `FollowJointTrajectory`:

```bash
ros2 launch my_arm_motion arm_pose_numeric_ik.launch.py \
  use_sim_time:=true \
  target_xyz:="[0,-400,500]" \
  target_rpy:="[90,0,0]" \
  seed_from_joint_states:=false \
  seed_joints:="[-60,-60,-100,170,-90,0]" \
  duration_sec:=4.0 \
  execute:=true
```

The controller interpolates the joints from their current state to the IK solution. Therefore, the TCP does not necessarily follow a straight Cartesian line, and the path between the two configurations is not checked for collisions.

This node introduces the operation later repeated by `arm_pose_sequence.py`:

```text
arm_pose_numeric_ik: one Cartesian pose → numerical IK → one joint target

arm_pose_sequence: several YAML poses → numerical IK for each pose
                  → consecutive joint targets
```

Both nodes require MoveIt's `/compute_ik` service, but neither uses the MoveIt motion planner. They are suitable for simple, known and previously validated movements.

<!-- Screenshot: numerical IK solution and direct single-pose execution. -->

## 3. Basic single motions

### 3.1 `arm_movej.py`

MoveJ calculates numerical IK for the target pose and uses OMPL to plan a
collision-aware joint-space path. The tool path is not necessarily straight.

Plan a movement to an approach pose:

```bash
ros2 launch my_arm_motion arm_movej.launch.py \
  use_sim_time:=true \
  target_xyz:="[0,-400,450]" \
  target_rpy:="[90,0,0]" \
  seed_from_joint_states:=true \
  avoid_collisions:=true \
  joint_tolerance:=0.001 \
  max_velocity:=0.1 \
  max_acceleration:=0.1 \
  motion_timeout_sec:=180.0 \
  execute:=false
```

After successful planning, repeat the command with `execute:=true`.

<!-- Screenshot: planned MoveJ in RViz2. -->

### 3.2 `arm_movel.py`

MoveL interpolates a straight Cartesian path from the current tool pose to the
target. It cannot curve around an obstacle while remaining a strict MoveL.

Starting from `[0,-400,450]`, plan a 50 mm downward motion:

```bash
ros2 launch my_arm_motion arm_movel.launch.py \
  use_sim_time:=true \
  target_xyz:="[0,-400,400]" \
  target_rpy:="[90,0,0]" \
  max_step:=0.005 \
  fraction_threshold:=1.0 \
  jump_threshold:=0.0 \
  avoid_collisions:=true \
  max_velocity:=0.1 \
  max_acceleration:=0.1 \
  execute:=false
```

The expected result is `fraction: 1.000`. Repeat with `execute:=true` only
after the complete path has been accepted.

<!-- Screenshot: straight MoveL path and fraction 1.000. -->

## 4. Singularity-checked motions

### 4.1 `arm_movej_sing.py`

This node extends MoveJ with a sampled numerical Jacobian analysis. It rejects
a trajectory when the smallest singular value is too low, the condition number
is too high, or a joint jump exceeds the configured limit.

```bash
ros2 launch my_arm_motion arm_movej_sing.launch.py \
  use_sim_time:=true \
  target_xyz:="[0,-400,500]" \
  target_rpy:="[90,0,0]" \
  seed_from_joint_states:=false \
  seed_joints:="[-60,-60,-100,170,-90,0]" \
  avoid_collisions:=true \
  singularity_samples:=20 \
  min_singular_value:=0.008 \
  max_condition_number:=250.0 \
  max_joint_jump_deg:=45.0 \
  execute:=false
```

### 4.2 `arm_movel_sing.py`

This node applies the same validation to every accepted Cartesian trajectory.
The following example assumes that the robot starts at `[0,-400,450]`:

```bash
ros2 launch my_arm_motion arm_movel_sing.launch.py \
  use_sim_time:=true \
  target_xyz:="[0,-400,400]" \
  target_rpy:="[90,0,0]" \
  max_step:=0.005 \
  fraction_threshold:=1.0 \
  avoid_collisions:=true \
  singularity_samples:=20 \
  min_singular_value:=0.008 \
  max_condition_number:=250.0 \
  max_joint_jump_deg:=45.0 \
  execute:=false
```

`singularity_samples:=0` checks every trajectory point but can be much slower.

<!-- Screenshot: singularity metrics and maximum joint jump. -->

## 5. Planning candidates and obstacles

### 5.1 `arm_movej_candidates.py`

This node perturbs the IK seed, searches for different IK branches, requests
several OMPL plans for each branch, evaluates the valid trajectories and saves
the best one.

Plan and save a MoveJ without an obstacle:

```bash
ros2 launch my_arm_motion arm_movej_candidates.launch.py \
  use_sim_time:=true \
  target_xyz:="[300,-200,400]" \
  target_rpy:="[90,0,0]" \
  ik_candidates:=6 \
  plans_per_ik:=4 \
  seed_perturbation_deg:=60.0 \
  check_singularities:=false \
  avoid_collisions:=true \
  trajectory_filename:=movej_without_obstacle.yaml \
  save_trajectory:=true \
  execute:=false
```

The singularity check is disabled here to make the obstacle comparison faster.
For final validation, use `check_singularities:=true` and suitable limits.

### 5.2 `arm_test_scene.py`

Add a box touching the ground. The centre height is half the box height:

```bash
ros2 launch my_arm_motion arm_test_scene.launch.py \
  use_sim_time:=true \
  operation:=add \
  object_id:=moveit_test_box \
  box_xyz:="[100,-400,275]" \
  box_size:="[100,140,550]"
```

Repeat the MoveJ candidate experiment and save it under another name:

```bash
ros2 launch my_arm_motion arm_movej_candidates.launch.py \
  use_sim_time:=true \
  target_xyz:="[300,-200,400]" \
  target_rpy:="[90,0,0]" \
  ik_candidates:=6 \
  plans_per_ik:=4 \
  seed_perturbation_deg:=60.0 \
  check_singularities:=false \
  avoid_collisions:=true \
  trajectory_filename:=movej_with_obstacle.yaml \
  save_trajectory:=true \
  execute:=false
```

Remove the object after the experiment:

```bash
ros2 launch my_arm_motion arm_test_scene.launch.py \
  use_sim_time:=true \
  operation:=remove \
  object_id:=moveit_test_box
```

<!-- Screenshot: box in RViz2 and valid/failed MoveJ candidates. -->

### 5.3 `arm_movel_candidates.py`

This node repeats Cartesian planning with scaled `max_step` values. It can find
a better discretisation, but it cannot create a curved detour around an obstacle.

Starting from `[0,-400,450]`:

```bash
ros2 launch my_arm_motion arm_movel_candidates.launch.py \
  use_sim_time:=true \
  target_xyz:="[0,-400,400]" \
  target_rpy:="[90,0,0]" \
  max_step:=0.005 \
  fraction_threshold:=1.0 \
  candidate_attempts:=3 \
  max_step_scales:="[1.0,0.75,0.5]" \
  check_singularities:=true \
  singularity_samples:=10 \
  min_singular_value:=0.008 \
  max_condition_number:=250.0 \
  avoid_collisions:=true \
  trajectory_filename:=movel_without_obstacle.yaml \
  save_trajectory:=true \
  execute:=false
```

## 6. YAML motion sequences

### 6.1 `arm_pose_sequence.py`: direct controller sequence

This legacy/simple node reads `common` and `steps` from YAML, calculates one IK
solution per pose, and sends each joint target directly to
`FollowJointTrajectory`. It is fast, but it does not plan or collision-check the
path between poses.

```bash
ros2 launch my_arm_motion arm_pose_sequence.launch.py \
  use_sim_time:=true \
  sequence_file:=ur5e_handshake.yaml
```

Execution is controlled by the YAML field `common.execute`.

### 6.2 `arm_motion_sequence.py`: planned sequence

This node reads a sequence of MoveJ and MoveL segments. It plans and validates
each segment, uses the end state of one segment as the start state of the next,
and optionally executes the segments in order. It does not save the sequence.

```bash
ros2 launch my_arm_motion arm_motion_sequence.launch.py \
  use_sim_time:=true \
  sequence_file:=ur5e_movej_movel.yaml \
  avoid_collisions:=true \
  check_singularities:=true \
  singularity_samples:=20 \
  min_singular_value:=0.008 \
  max_condition_number:=250.0 \
  execute:=false
```

After validation, repeat with `execute:=true`.

### 6.3 `arm_motion_sequence_saved.py`: concatenated saved sequence

This extended node plans all segments, validates them, applies the durations
defined in the YAML, concatenates them into one trajectory and optionally saves
the result. The tested handshake contains two MoveJ segments and three MoveL
segments.

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

Expected output:

```text
Concatenated 5 motions into 260 trajectory points; duration=19.000 s.
Saved concatenated trajectory to: .../ur5e_handshake_planned.yaml
```

If a segment is accelerated to a requested YAML duration shorter than the
MoveIt duration, verify that the resulting velocity and acceleration remain
acceptable before execution.

<!-- Screenshot: planned handshake sequence and saved YAML result. -->

## 7. `arm_execute_saved.py`: replay a saved trajectory

This node loads the exact saved trajectory, verifies that the current joints
match its first waypoint, and executes it without replanning. The Planning Scene
must remain compatible with the scene used during planning.

```bash
ros2 launch my_arm_motion arm_execute_saved.launch.py \
  use_sim_time:=true \
  trajectory_filename:=ur5e_handshake_planned.yaml \
  start_tolerance_deg:=5.0 \
  execute:=true
```

In the handshake experiment, execution was rejected with a start-state error of
`169.608 deg`. After restoring the original start state, the error was
`0.010 deg` and the 260-point sequence executed successfully.

Do not increase `start_tolerance_deg` merely to bypass a mismatch. Restore the
original start state or replan from the current state.

<!-- Screenshot: rejected start state and successful saved execution. -->

## 8. Main parameters

| Parameter | Meaning | Typical value |
|---|---|---|
| `target_xyz` | Cartesian target in millimetres | `[0,-400,450]` |
| `target_rpy` | RPY target in degrees | `[90,0,0]` |
| `execute` | Execute after successful validation | `false` first |
| `avoid_collisions` | Use collision checking | `true` |
| `max_velocity`, `max_acceleration` | MoveIt scaling factors | `0.1` |
| `fraction_threshold` | Minimum accepted MoveL fraction | `1.0` |
| `max_step` | Cartesian interpolation step | `0.005 m` |
| `singularity_samples` | Sampled trajectory points; `0` checks all | `10` or `20` |
| `min_singular_value` | Minimum accepted Jacobian singular value | `0.008` |
| `max_condition_number` | Maximum accepted condition number | `250.0` |
| `max_joint_jump_deg` | Maximum point-to-point joint jump | `45.0 deg` |
| `trajectory_filename` | YAML name inside the installed trajectory folder | descriptive `.yaml` name |
| `start_tolerance_deg` | Maximum saved-trajectory start error | `5.0 deg` |

The numerical singularity thresholds are empirical indicators and must be
validated for each robot model.

## 9. Recommended workflow

1. Start Gazebo, `move_group` and RViz2.
2. Confirm `/joint_states` and the controllers.
3. Plan with `execute:=false` and low scaling factors.
4. Require `fraction=1.0` for a complete MoveL.
5. Inspect collisions, singularity metrics and joint jumps.
6. Save candidates under descriptive filenames.
7. Before saved replay, restore the original start state and Planning Scene.
8. Execute in simulation before considering a real robot.

## 10. Conclusions

- MoveJ plans in joint space; MoveL follows a straight Cartesian path.
- The `_sing` nodes add numerical singularity and joint-jump validation.
- Candidate nodes reuse the `_sing` functionality and search for a better valid trajectory.
- Only objects in the MoveIt Planning Scene participate in collision checking.
- `arm_pose_sequence` is faster because it bypasses path planning, but it is less safe.
- `arm_motion_sequence` plans and validates YAML MoveJ/MoveL sequences.
- `arm_motion_sequence_saved` concatenates and saves a complete validated sequence.
- Saved trajectories require the original start state and are executed without replanning.
