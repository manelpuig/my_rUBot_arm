# MoveIt2 Motion Planning Validation

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

It searches for many possible trajectories and selects the best one
according to several constraints.

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

# 4. Laboratory roadmap

``` text
Experiment 1
MoveJ planning

↓

Experiment 2
MoveJ + Singularity analysis

↓

Experiment 3
MoveL planning

↓

Experiment 4
MoveL + Singularity analysis

↓

Experiment 5
Planning Scene

↓

Experiment 6
MoveJ with obstacle

↓

Experiment 7
Save trajectory

↓

Experiment 8
Execute saved trajectory
```

------------------------------------------------------------------------

# 5. Experiment 1 --- MoveJ planning

## Goal

Understand the MoveIt planning pipeline without considering
singularities.

Run:

``` bash
ros2 launch my_arm_motion arm_movej_candidates.launch.py \
  use_sim_time:=true \
  target_xyz:="[300,-200,400]" \
  target_rpy:="[90,0,0]" \
  ik_candidates:=2 \
  plans_per_ik:=2 \
  seed_perturbation_deg:=30.0 \
  check_singularities:=false \
  avoid_collisions:=true \
  execute:=false
```

Observe:

-   IK candidates
-   OMPL plans
-   Selected trajectory
-   Saved YAML file

> Insert terminal output and screenshots.

Discussion:

-   Why are there several valid trajectories?
-   Why does MoveIt select only one?

------------------------------------------------------------------------

# 6. Experiment 2 --- MoveJ with singularity analysis

Repeat the previous experiment using:

``` text
check_singularities:=true
min_singular_value:=0.01
max_condition_number:=200
```

Observe:

-   Rejected candidates
-   Accepted candidates
-   sigma_min
-   condition number

Discussion:

-   Why are some trajectories rejected?
-   How does singularity analysis improve safety?

------------------------------------------------------------------------

# 7. Experiment 3 --- MoveL planning

Run:

``` bash
ros2 launch my_arm_motion arm_movel_candidates.launch.py \
  use_sim_time:=true \
  target_xyz:="[300,-200,400]" \
  target_rpy:="[90,0,0]" \
  max_step:=0.005 \
  fraction_threshold:=1.0 \
  check_singularities:=false \
  execute:=false
```

Observe:

-   Straight TCP path
-   Cartesian interpolation
-   Planning fraction

Discussion:

-   Why are there fewer candidates than in MoveJ?

------------------------------------------------------------------------

# 8. Experiment 4 --- MoveL with singularity analysis

Run the same trajectory but enable singularity checking.

Try several target positions until a wrist singularity is detected.

Observe:

-   sigma_min
-   condition number
-   rejected Cartesian trajectories

Discussion:

Why can't MoveL modify the Cartesian line to avoid the singularity?

------------------------------------------------------------------------

# 9. Experiment 5 --- Planning Scene

Add a collision object.

Verify that the obstacle appears in RViz.

> Insert screenshot.

------------------------------------------------------------------------

# 10. Experiment 6 --- MoveJ with obstacle

Plan the same motion with the obstacle present.

Observe:

-   Collision-free trajectory
-   Selected candidate
-   Different joint-space path

Discussion:

Why can MoveJ avoid the obstacle?

------------------------------------------------------------------------

# 11. Experiment 7 --- Save trajectory

Verify the generated YAML trajectory.

Check:

-   metadata
-   joint names
-   trajectory points

> Insert YAML excerpt.

------------------------------------------------------------------------

# 12. Experiment 8 --- Execute saved trajectory

Run:

``` bash
ros2 launch my_arm_motion arm_execute_saved.launch.py \
  use_sim_time:=true \
  execute:=true
```

Observe:

-   No IK computation
-   No OMPL planning
-   Trajectory replay

Discussion:

Why is planning separated from execution?

------------------------------------------------------------------------

# 13. Questions

1.  Why can MoveJ generate many trajectories?
2.  Why is MoveL more restrictive?
3.  Why can't MoveL avoid a wrist singularity?
4.  What information is stored in the YAML file?
5.  Why is the Planning Scene important?
6.  Why are planning and execution separated?

------------------------------------------------------------------------

# 14. Main conclusions

MoveIt is not only a motion execution tool.

Its main strength is its ability to:

-   search,
-   evaluate,
-   compare,
-   reject,
-   select,

the best trajectory before execution.

These capabilities are essential for modern ROS 2 robotic applications.
