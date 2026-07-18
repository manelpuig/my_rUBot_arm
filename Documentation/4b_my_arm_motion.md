# Arm motion with MoveIt 2

In the previous document, we created and configured the MoveIt 2 packages for the different robot arms.

MoveIt now knows:

- the robot model from the URDF;
- the planning groups and semantic information from the SRDF;
- the numerical inverse kinematics solver;
- the joint limits;
- the self-collision rules;
- the trajectory controller.

In this document, we will use the `my_arm_motion` package to send Cartesian targets to the robot and execute movements.

We will study:

- numerical inverse kinematics;
- the effect of the IK seed;
- motion planning to one Cartesian pose;
- trajectory execution;
- pose sequences defined in YAML files;
- collision checking and the current limitations of the examples;
- unreachable targets, singularities and planning errors.

## 1. The `my_arm_motion` package

The `my_arm_motion` package contains generic ROS 2 nodes for different 6-DOF robot arms.

The same nodes can be used with the PUMA and UR5e robots because both MoveIt configuration packages use:

- the planning group `arm`;
- the end-effector link `tool`;
- the base frame `base_link`;
- the joint names `joint1` to `joint6`;
- the controller action `/arm_controller/follow_joint_trajectory`.

The package provides two main executables:

| Executable | Launch file | Purpose |
|---|---|---|
| `arm_pose_exe` | `arm_pose.launch.py` | Move the robot to one Cartesian pose |
| `arm_pose_sequence_exe` | `arm_pose_sequence.launch.py` | Execute a sequence of Cartesian poses from a YAML file |

The package structure is:

```text
my_arm_motion/
├── config/
│   ├── puma_handshake.yaml
│   └── ur5e_handshake.yaml
├── launch/
│   ├── arm_pose.launch.py
│   └── arm_pose_sequence.launch.py
└── my_arm_motion/
    ├── arm_pose.py
    └── arm_pose_sequence.py
```

## 2. Motion flow

For a single Cartesian target, the general flow is:

```text
Cartesian target [x, y, z, roll, pitch, yaw]
                       ↓
Transform target to the planning frame
                       ↓
MoveIt `/compute_ik` service
                       ↓
Kinematically valid joint configuration
                       ↓
MoveIt joint-space motion planning
                       ↓
Trajectory execution
                       ↓
FollowJointTrajectory controller
                       ↓
Gazebo or real robot
```

It is important to distinguish three operations:

1. **Inverse kinematics:** finds joint values for the desired end-effector pose.
2. **Motion planning:** finds a path from the current joint configuration to the target configuration.
3. **Trajectory execution:** sends the planned movement to the robot controller.

A valid IK solution does not automatically mean that the solution is collision-free or that a valid path exists.

## 3. Start the simulation

Open one terminal for the simulated robot.

### 3.1 PUMA

```bash
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py \
  use_sim_time:=true \
  model:=my_arm_puma.urdf.xacro \
  controllers:=gz_controllers.yaml \
  use_gripper:=false
```

### 3.2 UR5e

```bash
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py \
  use_sim_time:=true \
  model:=my_arm_ur5e.urdf.xacro \
  controllers:=gz_controllers.yaml \
  use_gripper:=false
```

Check that the robot is visible in Gazebo and that the controllers are active.

Useful commands are:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 action list
```

The expected arm action is:

```text
/arm_controller/follow_joint_trajectory
```

## 4. Start MoveIt 2 and RViz2

Open a second terminal and start the MoveIt `move_group` node.

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

Open a third terminal for RViz2.

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

RViz2 is not required by the motion node, but it is very useful for observing:

- the current robot configuration;
- the target configuration;
- the planned trajectory;
- self-collisions;
- obstacles in the Planning Scene.

## 5. Define a Cartesian target

A Cartesian pose contains a position and an orientation.

The launch file uses:

- `target_xyz`: position `[x, y, z]` in millimetres;
- `target_rpy`: orientation `[roll, pitch, yaw]` in degrees.

For example:

```text
target_xyz:=[140, -800, 300]
target_rpy:=[0, 70, -90]
```

The launch file converts:

- millimetres to metres;
- degrees to radians.

The Python node converts roll, pitch and yaw to a quaternion before sending the pose to MoveIt.

By default, the target pose and the MoveIt planning frame are both `base_link`.

## 6. Move to one Cartesian pose

The `arm_pose.py` node performs the following operations:

1. reads the Cartesian target;
2. transforms it to the MoveIt planning frame using TF2;
3. selects the IK seed;
4. calls the MoveIt `/compute_ik` service;
5. prints the resulting joint configuration;
6. asks MoveIt to move to that joint configuration;
7. waits until execution finishes;
8. closes the node.

### 6.1 PUMA example

```bash
ros2 launch my_arm_motion arm_pose.launch.py \
  use_sim_time:=true \
  target_xyz:="[140,-800,300]" \
  target_rpy:="[0.0,70.0,-90.0]" \
  seed_from_joint_states:=false \
  seed_joints:="[-90,-40,30,0,0,0]" \
  execute:=true
```

### 6.2 UR5e example

```bash
ros2 launch my_arm_motion arm_pose.launch.py \
  use_sim_time:=true \
  target_xyz:="[0,-400,500]" \
  target_rpy:="[90.0,0.0,0.0]" \
  seed_from_joint_states:=false \
  seed_joints:="[-60,-60,-100,170,-90,0]" \
  execute:=true
```

## 7. Main launch parameters

| Parameter | Meaning | Units or values |
|---|---|---|
| `target_xyz` | Desired end-effector position | `[x,y,z]` in mm |
| `target_rpy` | Desired end-effector orientation | `[roll,pitch,yaw]` in degrees |
| `seed_from_joint_states` | Use the current robot state as the IK seed | `true` or `false` |
| `seed_joints` | Fallback or manually selected IK seed | Six joint angles in degrees |
| `execute` | Execute the motion after finding the IK solution | `true` or `false` |
| `max_velocity` | Velocity scaling factor | Normally between `0.0` and `1.0` |
| `max_acceleration` | Acceleration scaling factor | Normally between `0.0` and `1.0` |
| `ik_timeout_sec` | Maximum time for the IK request | Seconds |
| `print_joints` | Print the IK solution | `true` or `false` |
| `use_sim_time` | Use the Gazebo clock | `true` in simulation |

The default velocity and acceleration scaling factors in the launch file are `0.2`.

Start with low values, especially before using a real robot.

## 8. Understand the IK seed

A numerical IK solver needs an initial joint configuration called the **seed**.

The solver normally searches for a solution near this configuration. Therefore, different seeds can produce different IK solutions for the same Cartesian pose.

The seed can affect:

- the elbow-up or elbow-down configuration;
- the wrist orientation;
- the distance from joint limits;
- convergence of the numerical solver;
- the final path of the robot.

### 8.1 Use the current robot state

```bash
seed_from_joint_states:=true
```

The node reads `/joint_states` and uses the current positions of `joint1` to `joint6`.

This usually produces a solution close to the current robot configuration and reduces sudden configuration changes.

### 8.2 Use a manual seed

```bash
seed_from_joint_states:=false \
seed_joints:="[-90,-40,30,0,0,0]"
```

The manual values are written in degrees. The launch file converts them to radians.

A manual seed is useful when:

- the current configuration produces an unwanted IK branch;
- the numerical solver does not converge;
- we want a specific elbow or wrist configuration;
- we want reproducible results.

### 8.3 Suggested experiment

Keep the same Cartesian target and execute the node with two different manual seeds.

Compare:

- the joint values printed by the node;
- the final configuration in RViz2;
- the robot movement in Gazebo;
- whether both seeds find a valid solution.

## 9. Calculate IK without moving the robot

Before executing a new target, it is useful to test only the inverse kinematics:

```bash
ros2 launch my_arm_motion arm_pose.launch.py \
  use_sim_time:=true \
  target_xyz:="[140,-800,300]" \
  target_rpy:="[0.0,70.0,-90.0]" \
  seed_from_joint_states:=false \
  seed_joints:="[-90,-40,30,0,0,0]" \
  execute:=false \
  print_joints:=true
```

With `execute:=false`, the node:

- calculates the IK solution;
- prints the joint values;
- exits without moving the robot.

This is the recommended first test for a new Cartesian target.

## 10. What is planned for a single pose?

After receiving a valid IK solution, `arm_pose.py` calls:

```python
self.moveit2.move_to_configuration(joint_goal)
```

MoveIt plans a movement in joint space from the current robot configuration to the IK joint goal.

During planning, MoveIt can consider:

- joint limits;
- self-collisions;
- collision objects in the Planning Scene;
- the selected planning algorithm;
- velocity and acceleration scaling.

The end effector does not necessarily follow a straight Cartesian line. The planner generates a valid joint-space path.

The current node does not request collision checking inside the `/compute_ik` call. Collision checking is performed later when MoveIt plans the movement to the IK joint goal. It is therefore possible to obtain an IK solution but fail during motion planning.

If a straight Cartesian path is required, a Cartesian-path planning method must be implemented explicitly.

## 11. Execute a sequence of poses

The `arm_pose_sequence.py` node reads several Cartesian targets from a YAML file and executes them in order.

### 11.1 PUMA handshake

```bash
ros2 launch my_arm_motion arm_pose_sequence.launch.py \
  use_sim_time:=true \
  sequence_file:=puma_handshake.yaml
```

### 11.2 UR5e handshake

```bash
ros2 launch my_arm_motion arm_pose_sequence.launch.py \
  use_sim_time:=true \
  sequence_file:=ur5e_handshake.yaml
```

The launch file finds the selected YAML file inside the installed `my_arm_motion/config` directory.

When the complete sequence finishes, the launch system closes automatically.

## 12. YAML sequence structure

A sequence file contains two sections:

- `common`: parameters shared by all steps;
- `steps`: the Cartesian targets executed in order.

Example:

```yaml
common:
  execute: true
  group_name: arm
  ik_link: tool
  target_frame: base_link
  planning_frame: base_link
  ik_timeout_sec: 1.0
  print_joints: true

steps:
  - name: home_start
    target_xyz: [140, -850, 400]
    target_rpy: [0.0, 50.0, -90.0]
    seed_from_joint_states: false
    seed_joints: [-90, -40, 30, 0, 0, 0]
    duration: 4.0
    sleep_after: 1.0

  - name: approach_handshake
    target_xyz: [140, -800, 300]
    target_rpy: [0.0, 70.0, -90.0]
    seed_from_joint_states: true
    duration: 2.0
    sleep_after: 1.0
```

### Common parameters

| Parameter | Meaning |
|---|---|
| `execute` | Execute the trajectories or calculate IK only |
| `group_name` | MoveIt planning group |
| `ik_link` | Link that must reach the target pose |
| `target_frame` | Frame used to define the targets |
| `planning_frame` | Frame used by MoveIt |
| `ik_timeout_sec` | IK timeout for every step |
| `print_joints` | Print the joint solution for every step |
| `seed_from_joint_states` | Default seed method, if defined |
| `seed_joints` | Default manual seed, if defined |
| `duration` | Default movement duration, if defined |

### Step parameters

| Parameter | Meaning |
|---|---|
| `name` | Descriptive name of the step |
| `target_xyz` | Cartesian position in mm |
| `target_rpy` | Cartesian orientation in degrees |
| `seed_from_joint_states` | Seed method for this step |
| `seed_joints` | Optional manual seed for this step |
| `duration` | Time assigned to reach the joint target |
| `sleep_after` | Pause after completing the movement |

A parameter defined inside a step overrides the corresponding default value in `common`.

## 13. How the current sequence node works

For every YAML step, the sequence node:

1. converts millimetres to metres and degrees to radians;
2. creates a Cartesian pose;
3. transforms the pose to the planning frame;
4. selects the current or manual IK seed;
5. calls `/compute_ik`;
6. obtains the target values for `joint1` to `joint6`;
7. creates one `JointTrajectoryPoint`;
8. sends it directly to `/arm_controller/follow_joint_trajectory`;
9. waits for the result;
10. starts the next step.

### Important current limitation

The current sequence node uses MoveIt for numerical IK, but it does **not** ask a MoveIt motion planner to calculate the path between the sequence poses.

It sends each IK joint goal directly to the trajectory controller as a single trajectory point.

The current `/compute_ik` request also does not enable collision checking. Therefore, the sequence implementation does not guarantee that the target configuration or the path between poses is collision-free. The interpolated movement could pass through an obstacle, a self-collision, a singular configuration or an undesired robot configuration.

Use these sequences first in simulation and in an environment without obstacles.

## 14. Create a new movement sequence

To create a new social or functional movement:

1. copy one of the existing YAML files;
2. give the new file a descriptive name;
3. define an initial safe pose;
4. add small movements between consecutive targets;
5. use `seed_from_joint_states: true` for continuous movements;
6. use a manual seed when a specific configuration is required;
7. assign conservative durations;
8. test every pose with `execute: false`;
9. test the complete sequence in Gazebo;
10. only then consider testing it with the real robot.

After adding a new YAML file, rebuild the package so that the file is copied to the install directory:

```bash
colcon build --symlink-install --packages-select my_arm_motion
source install/setup.bash
```

## 15. Collision checking and obstacles

MoveIt can check:

- self-collisions between robot links;
- collisions with the floor, table or other objects;
- collisions with attached objects;
- joint-limit violations.

However, MoveIt only knows the objects included in its **Planning Scene**.

An object visible in Gazebo is not automatically known by MoveIt. The object must also be added to the MoveIt Planning Scene with the correct shape, size and pose.

The current repository examples do not yet add custom obstacles from `my_arm_motion`.

A future extension can add:

- a table as a collision box;
- a wall or safety area;
- an object to pick;
- an object attached to the gripper;
- a sequence in which every movement is planned by MoveIt.

## 16. Singularities

A singularity is a robot configuration where the end effector loses one or more independent motion directions or where very large joint velocities may be required for a small Cartesian movement.

Near a singularity:

- numerical IK may fail or converge slowly;
- small target changes may produce large joint changes;
- different seeds may produce very different solutions;
- Cartesian motion may become unstable or impractical.

Standard MoveIt planning does not automatically guarantee that every singular configuration is avoided.

Practical methods to reduce problems are:

- choose targets away from fully extended configurations;
- avoid wrist alignments known to be singular;
- use the current joint state as the seed for continuous motions;
- compare consecutive IK solutions;
- reject solutions with very large joint changes;
- test the trajectory in RViz2 and Gazebo;
- use low velocity and acceleration values.

## 17. Common errors and possible causes

### Waiting for `/compute_ik`

Possible cause: the MoveIt `move_group` node is not running.

Check:

```bash
ros2 service list | grep compute_ik
```

### Waiting for `/joint_states`

Possible causes:

- Gazebo is not running;
- the joint-state broadcaster is not active;
- joint names do not match `joint1` to `joint6`.

Check:

```bash
ros2 topic echo /joint_states --once
```

### IK failed

Possible causes:

- the position is outside the workspace;
- the orientation is impossible;
- the seed is not appropriate;
- the target is close to a singularity;
- the IK timeout is too short;
- the planning group or IK link is incorrect.

Try:

- a target closer to the robot;
- a different orientation;
- another seed;
- a larger `ik_timeout_sec`;
- `execute:=false` to test only the IK.

### Trajectory goal rejected

Possible causes:

- the arm controller is not active;
- the action name is incorrect;
- the joint names do not match;
- a target violates a joint limit;
- the trajectory duration is too short.

Check:

```bash
ros2 control list_controllers
ros2 action info /arm_controller/follow_joint_trajectory -t
```

### Robot configuration changes unexpectedly

Possible causes:

- the IK solver selected another solution branch;
- the manual seed is far from the current robot state;
- the target is close to a singularity.

Try `seed_from_joint_states:=true` or select a better manual seed.

### TF transformation error

Possible causes:

- `target_frame` or `planning_frame` is incorrect;
- the required transform is not being published;
- the robot description node is not running.

Check:

```bash
ros2 run tf2_ros tf2_echo base_link tool
```

## 18. Recommended test procedure

For every new target or sequence:

1. verify that Gazebo, controllers and MoveIt are running;
2. calculate IK with motion disabled;
3. inspect the printed joint values;
4. check that the values are inside reasonable limits;
5. plan and execute one pose in simulation;
6. observe the complete movement in RViz2 and Gazebo;
7. test the complete YAML sequence in simulation;
8. use low velocity and long durations;
9. stop if the robot changes configuration unexpectedly;
10. validate the environment before using a real robot.

## 19. Current implementation and future improvements

The current package provides a simple progression:

| Example | IK | MoveIt planning | Execution |
|---|---|---|---|
| `arm_pose.py` | MoveIt `/compute_ik` | Yes, to the IK joint goal | Through MoveIt and `ros2_control` |
| `arm_pose_sequence.py` | MoveIt `/compute_ik` for every step | No path planning between steps | Direct `FollowJointTrajectory` goals |

Possible future improvements are:

- use MoveIt planning for every YAML step;
- preview a complete sequence before execution;
- add collision objects to the Planning Scene;
- implement Cartesian paths;
- compare different OMPL planners;
- reject large joint jumps between consecutive IK solutions;
- include singularity or manipulability checks;
- control a gripper inside the YAML sequence;
- use the same application with a real robot.

## 20. Main conclusions

- Analytical IK is useful for understanding robot geometry.
- Numerical IK provides a general solution for different robot arms.
- The IK seed influences the solution found by the numerical solver.
- IK calculates a target configuration, but motion planning calculates how to reach it.
- MoveIt can plan collision-aware movements only when the Planning Scene is correctly defined.
- The current single-pose node uses MoveIt motion planning.
- The current sequence node uses MoveIt IK but sends the joint targets directly to the controller.
- Simulation and conservative motion parameters are essential before using a real robot.
