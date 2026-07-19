# MoveIt2 Motion Planning Validation

## Why MoveIt? Collision Avoidance and Singularity Analysis

> Laboratory guide for **my_rUBot_arm**

------------------------------------------------------------------------

# 1. Why MoveIt if RoboDK already works?

Many students ask:

> **If RoboDK already executes MoveJ and MoveL, why do we need MoveIt?**

This laboratory answers that question experimentally.

## RoboDK

RoboDK is an excellent offline programming tool.

It can:

-   Compute Forward Kinematics (FK).
-   Compute Inverse Kinematics (IK).
-   Execute MoveJ and MoveL.
-   Simulate industrial robots.
-   Generate robot programs.

However, RoboDK normally assumes that the user defines a safe
trajectory.

It is not designed to continuously search among many possible robot
motions while considering a changing environment.

------------------------------------------------------------------------

## MoveIt

MoveIt is a motion planning framework.

Instead of executing a single trajectory, it searches for many possible
trajectories and selects one that satisfies several constraints.

Typical planning pipeline:

``` text
Target Pose
      │
      ▼
Inverse Kinematics
      │
      ▼
Several IK candidates
      │
      ▼
OMPL planners
      │
      ▼
Many trajectories
      │
      ▼
Collision checking
      │
      ▼
Singularity analysis
      │
      ▼
Trajectory scoring
      │
      ▼
Best trajectory
      │
      ▼
Execute
```

MoveIt can automatically:

-   avoid obstacles,
-   reject collisions,
-   reject singular trajectories,
-   compare different solutions,
-   save a trajectory,
-   execute it later.

------------------------------------------------------------------------

# 2. Learning objectives

After this laboratory the student should understand:

-   Difference between MoveJ and MoveL.
-   Difference between joint-space and Cartesian planning.
-   What a Planning Scene is.
-   Why singularities are dangerous.
-   Why MoveJ can avoid obstacles while MoveL cannot.
-   Why planning and execution should be separated.

------------------------------------------------------------------------

# 3. Laboratory setup

Launch:

-   Gazebo / Fake Hardware
````bash
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py \
  use_sim_time:=true \
  model:=my_arm_ur5e.urdf.xacro \
  controllers:=gz_controllers.yaml \
  use_gripper:=false
````
-   MoveIt
````bash
ros2 launch ur5e_moveit_config move_group.launch.py \
  use_sim_time:=true
````
-   RViz
````bash
ros2 launch ur5e_moveit_config moveit_rviz.launch.py \
  use_sim_time:=true
````

Move the robot to **Ready**.
![](../Images/ur5e_ready_gazebo.png)
![](../Images/ur5e_ready_gazebo.png)

Do **not** use Home because the current Home pose is close to a wrist
singularity.

------------------------------------------------------------------------

# 4. Experiment 1 --- MoveJ without obstacles

## Goal

Verify normal MoveJ planning.

Launch

``` bash
ros2 launch my_arm_motion arm_movej_candidates.launch.py \
  use_sim_time:=true \
  target_xyz:="[300.0,-200.0,400.0]" \
  target_rpy:="[90.0,0.0,0.0]" \
  ik_candidates:=2 \
  plans_per_ik:=2 \
  seed_perturbation_deg:=30.0 \
  check_singularities:=true \
  singularity_samples:=10 \
  min_singular_value:=0.01 \
  max_condition_number:=200.0 \
  max_joint_jump_deg:=45.0 \
  avoid_collisions:=true \
  trajectory_file:=/tmp/movej_no_obstacle.yaml \
  execute:=false
```

Observe:

-   IK candidates
-   OMPL candidates
-   selected trajectory
-   YAML file
```text
[arm_movej_candidates_exe-1] [INFO] [1784480714.655302014] [arm_movej_candidates]: Current joint state is available.
[arm_movej_candidates_exe-1] [INFO] [1784480714.665479732] [arm_movej_candidates]: Using /joint_states as IK seed.
[arm_movej_candidates_exe-1] [INFO] [1784480714.747241001] [arm_movej_candidates]: Searching up to 2 IK candidates and 2 OMPL plans per IK.
[arm_movej_candidates_exe-1] [INFO] [1784480714.854600821] [arm_movej_candidates]: Joint states are available now
[arm_movej_candidates_exe-1] [WARN] [1784480720.460467085] [arm_movej_candidates]: Candidate 1: rejected.
[arm_movej_candidates_exe-1] [INFO] [1784480720.461289316] [arm_movej_candidates]: Joint states are available now
[arm_movej_candidates_exe-1] [WARN] [1784480726.060052374] [arm_movej_candidates]: Candidate 2: rejected.
[arm_movej_candidates_exe-1] [INFO] [1784480726.162311767] [arm_movej_candidates]: Joint states are available now
[arm_movej_candidates_exe-1] [INFO] [1784480733.865091096] [arm_movej_candidates]: Candidate 3: valid, sigma_min=0.017513, condition=119.88, path=7.479, score=-0.218.
[arm_movej_candidates_exe-1] [INFO] [1784480733.865813493] [arm_movej_candidates]: Joint states are available now
[arm_movej_candidates_exe-1] [INFO] [1784480741.764207193] [arm_movej_candidates]: Candidate 4: valid, sigma_min=0.017502, condition=119.96, path=7.478, score=-0.220.
[arm_movej_candidates_exe-1] [INFO] [1784480741.764912462] [arm_movej_candidates]: Selected IK 2, plan 1: score=-0.218, sigma_min=0.017513, condition=119.88.
[arm_movej_candidates_exe-1] [INFO] [1784480742.746689875] [arm_movej_candidates]: Saved selected trajectory to: /tmp/movej_no_obstacle.yaml
[arm_movej_candidates_exe-1] [INFO] [1784480742.747528526] [arm_movej_candidates]: execute:=false -> selected trajectory was saved but not executed.
[INFO] [arm_movej_candidates_exe-1]: process has finished cleanly [pid 6942]
```
The stored trajectory:
```text
user:~/my_rUBot_arm$ ls -lh /tmp/movej_no_obstacle.yaml
-rw-r--r-- 1 user user 68K Jul 19 17:05 /tmp/movej_no_obstacle.yaml
user:~/my_rUBot_arm$ head -40 /tmp/movej_no_obstacle.yaml
metadata:
  motion_type: MoveJ
  planning_frame: base_link
  group_name: arm
  ik_link: tool
  target_xyz:
  - 0.3
  - -0.2
  - 0.4
  target_rpy:
  - 1.5707963267948966
  - 0.0
  - 0.0
  ik_candidate: 2
  ompl_plan: 1
  min_sigma: 0.017512800748780376
  max_condition: 119.87645373070312
  max_joint_jump_deg: 2.291831180524796
  joint_path_length: 7.47879904838889
  worst_index: 53
  score: -0.21828267907313048
trajectory:
  joint_names:
  - joint1
  - joint2
  - joint3
  - joint4
  - joint5
  - joint6
  points:
  - positions:
    - -1.5707133093051604
    - -1.7000059988651839
    - -1.7000579074424949
    - 0.19997701132338971
    - 1.5707407280670937
    - 8.45823198090961e-05
    velocities:
    - -0.0
    - 0.0
user:~/my_rUBot_arm$
```

### Discussion

Many trajectories reach exactly the same TCP pose.

MoveIt chooses one.

### Screenshots

-   Target pose
-   Planned trajectory
-   Console showing candidate scores

------------------------------------------------------------------------

# 5. Experiment 2 --- MoveL without obstacles

Run

``` bash
ros2 launch my_arm_motion arm_movel_candidates.launch.py \
  use_sim_time:=true \
  target_xyz:="[300.0,-200.0,400.0]" \
  target_rpy:="[90.0,0.0,0.0]" \
  max_step:=0.005 \
  fraction_threshold:=1.0 \
  candidate_attempts:=2 \
  max_step_scales:="[1.0,0.75,0.5]" \
  check_singularities:=false \
  avoid_collisions:=true \
  trajectory_file:=/tmp/movel_no_obstacle.yaml \
  execute:=false
```

Observe the TCP path.

Unlike MoveJ, MoveL must preserve one straight Cartesian line.

### Discussion

Why are there fewer candidates than in MoveJ?

### Screenshot

Straight TCP path.

------------------------------------------------------------------------

# 6. Experiment 3 --- Wrist singularity

Find a target producing

-   fraction = 1.000
-   singularity rejection

Example

``` bash
ros2 launch my_arm_motion arm_movel_sing.launch.py \
 use_sim_time:=true \
 target_xyz:="[0.0,400.0,400.0]" \
 target_rpy:="[90.0,0.0,0.0]" \
 singularity_samples:=0 \
 execute:=false
```

Try

-   Y=±400 mm
-   Y=±450 mm
-   Y=±500 mm

until the trajectory crosses a wrist singularity.

Repeat using

``` bash
ros2 launch my_arm_motion arm_movel_candidates.launch.py \
 use_sim_time:=true \
 target_xyz:="[0.0,400.0,400.0]" \
 target_rpy:="[90.0,0.0,0.0]" \
 execute:=false
```

Expected result:

All candidates rejected.

### Why?

MoveL cannot change the requested straight line.

If the line is singular, every candidate is singular.

### Screenshots

-   Straight line
-   Console showing sigma_min
-   Condition number

------------------------------------------------------------------------

# 7. Experiment 4 --- Add an obstacle

``` bash
ros2 launch my_arm_motion arm_test_scene.launch.py \
 use_sim_time:=true \
 operation:=add \
 box_xyz:="[0,-200,400]" \
 box_size:="[180,180,550]"
```

Verify the Planning Scene.

### Screenshot

Obstacle visible in RViz.

------------------------------------------------------------------------

# 8. Experiment 5 --- MoveJ with obstacle

Run

``` bash
ros2 launch my_arm_motion arm_movej_candidates.launch.py \
 use_sim_time:=true \
 target_xyz:="[0,-400,400]" \
 target_rpy:="[90,0,0]" \
 ik_candidates:=6 \
 plans_per_ik:=4 \
 execute:=false
```

Concept:

``` text
Requested TCP

Start -------- Target

      ███ obstacle

MoveJ

      ↗
Start     ↘ Target
```

OMPL searches another joint-space trajectory.

### Screenshots

-   Obstacle
-   Planned trajectory
-   Selected candidate

------------------------------------------------------------------------

# 9. Experiment 6 --- Save trajectory

Verify

``` text
/tmp/movej_obstacle.yaml
```

contains

-   metadata
-   joint names
-   trajectory points

### Screenshot

YAML file.

------------------------------------------------------------------------

# 10. Experiment 7 --- Execute without replanning

``` bash
ros2 launch my_arm_motion arm_execute_saved.launch.py \
 use_sim_time:=true \
 trajectory_file:=/tmp/movej_obstacle.yaml \
 start_tolerance_deg:=2.0
```

Observe

-   no IK
-   no OMPL
-   trajectory replay

### Screenshot

Execution console.

------------------------------------------------------------------------

# 11. Questions

1.  Why can MoveJ generate many trajectories?
2.  Why is MoveL much more restrictive?
3.  Why can't MoveL avoid the wrist singularity?
4.  What information is stored in the YAML file?
5.  Why is Planning Scene important?
6.  Why is planning separated from execution?

------------------------------------------------------------------------

# 12. MoveIt vs RoboDK

  Capability                       RoboDK    MoveIt
  ------------------------------- --------- --------
  FK                                  ✓        ✓
  IK                                  ✓        ✓
  MoveJ                               ✓        ✓
  MoveL                               ✓        ✓
  Automatic collision avoidance       ✗        ✓
  Planning Scene                      ✗        ✓
  OMPL planners                       ✗        ✓
  Multiple trajectory search          ✗        ✓
  Singularity analysis             Limited     ✓
  Store trajectory                 Limited     ✓
  Execute later                    Limited     ✓
  ROS2 integration                 Partial     ✓

------------------------------------------------------------------------

# 13. Main conclusions

This laboratory demonstrates that RoboDK and MoveIt are complementary.

RoboDK is an excellent offline programming environment.

MoveIt is a motion planning framework capable of making decisions
automatically.

The most important difference is not that both execute MoveJ or MoveL.

The important difference is that MoveIt can **search**, **evaluate**,
**compare** and **select** trajectories before execution.

These capabilities are essential in collaborative robotics, autonomous
manipulation and modern ROS2 applications.
