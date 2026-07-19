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
-   MoveIt
-   RViz

Move the robot to **Ready**.

Do **not** use Home because the current Home pose is close to a wrist
singularity.

### Screenshot 1

Robot in Ready pose.

### Screenshot 2

RViz MotionPlanning panel.

------------------------------------------------------------------------

# 4. Experiment 1 --- MoveJ without obstacles

## Goal

Verify normal MoveJ planning.

Launch

``` bash
ros2 launch my_arm_motion arm_movej_candidates.launch.py \
  use_sim_time:=true \
  target_xyz:="[0.40,0.00,0.40]" \
  target_rpy:="[90.0,0.0,0.0]" \
  ik_candidates:=4 \
  plans_per_ik:=3 \
  execute:=false
```

Observe:

-   IK candidates
-   OMPL candidates
-   selected trajectory
-   YAML file

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
 target_xyz:="[0.00,-0.40,0.40]" \
 target_rpy:="[90.0,0.0,0.0]" \
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
