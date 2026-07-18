# MoveIt 2 configuration packages

In the previous sections, we calculated the inverse kinematics of the PUMA robot using analytical equations.

Analytical inverse kinematics is useful because it provides a direct mathematical solution and helps us understand the geometry of the robot. However, it has some limitations:

- the equations are different for each robot;
- deriving the equations can be complex;
- not all robots have a practical analytical solution;
- robots with different structures or degrees of freedom require different equations.

For these reasons, we will now introduce **MoveIt 2** and **numerical inverse kinematics**.

## 1. Why do we use numerical inverse kinematics?

A numerical inverse kinematics solver starts from an initial joint configuration, called the **seed**, and iteratively searches for joint values that place the end effector near the desired Cartesian pose.

The main advantages are:

- the same general method can be used with different robots;
- it is not necessary to derive new analytical equations for every robot;
- the solver can consider joint limits;
- it can be integrated with collision checking and motion planning.

Numerical inverse kinematics also has some limitations:

- a solution is not always found;
- the result can depend on the initial seed;
- different seeds may produce different robot configurations;
- the target may be outside the robot workspace;
- convergence can be difficult near singular configurations.

In this repository, the MoveIt configuration packages use the numerical KDL kinematics plugin.

For example, the `kinematics.yaml` file contains:

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
```

> **Important:** A robot with 5 degrees of freedom cannot generally achieve every possible 3D position and orientation. If MoveIt cannot find a solution, the target pose may be incompatible with the kinematic structure of the robot.

## 2. What is MoveIt 2?

MoveIt 2 is the main ROS 2 framework for manipulation and robot-arm motion planning.

MoveIt does not replace the robot description, Gazebo or `ros2_control`. It connects these components and adds the tools required to calculate, plan and execute robot movements.

MoveIt 2 provides:

- numerical inverse kinematics;
- joint-limit checking;
- self-collision checking;
- collision checking with objects in the environment;
- motion planning between robot configurations;
- trajectory generation;
- predefined robot poses;
- planning and visualization in RViz2;
- trajectory execution through `ros2_control` controllers.

The general motion flow is:

```text
Cartesian target
       ↓
Numerical inverse kinematics
       ↓
Valid joint configuration
       ↓
Motion planner
       ↓
Collision-free joint trajectory
       ↓
FollowJointTrajectory action
       ↓
ros2_control controller
       ↓
Simulated or real robot
```

Inverse kinematics and motion planning are not the same operation:

- **Inverse kinematics** calculates a joint configuration for a Cartesian target.
- **Motion planning** calculates how the robot can move from its current configuration to the target configuration.

## 3. Analytical IK, numerical IK and motion planning

| Method | Main purpose | Advantages | Limitations |
|---|---|---|---|
| Analytical IK | Calculate joint values directly | Fast and mathematically clear | Robot-specific and sometimes difficult to derive |
| Numerical IK | Search iteratively for a joint solution | General and adaptable to different robots | Depends on the seed and may not converge |
| MoveIt motion planning | Find a complete path to the target | Checks limits and collisions and generates a trajectory | Requires a correct robot and environment configuration |

The three approaches are complementary. Analytical IK helps us understand robot geometry. Numerical IK provides a general method for different robots. MoveIt uses these tools as part of a complete motion-planning system.

## 4. Install MoveIt 2

This repository uses ROS 2 Humble.

Install MoveIt 2 with:

```bash
sudo apt update
sudo apt install ros-humble-moveit
```

The main documentation is available at:

- [MoveIt 2 installation](https://moveit.ai/install-moveit2/binary/)
- [MoveIt 2 documentation for ROS 2 Humble](https://moveit.picknik.ai/humble/index.html)

## 5. The MoveIt Setup Assistant

The **MoveIt Setup Assistant** is a graphical tool used to generate the first MoveIt configuration package for a robot.

Start it with:

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

The assistant loads the robot URDF and generates the semantic and configuration files required by MoveIt.

### 5.1 Load the robot model

Select **Create New MoveIt Configuration Package** and load the robot URDF.

If the robot description is written in Xacro, first generate a temporary URDF file. For example:

```bash
xacro path/to/robot.urdf.xacro > /tmp/robot_moveit.urdf
```

Load `/tmp/robot_moveit.urdf` in the Setup Assistant.

The URDF defines the physical robot structure:

- links and joints;
- joint types and limits;
- visual and collision geometry;
- the kinematic tree.

### 5.2 Generate the self-collision matrix

Open the **Self-Collisions** section and generate the collision matrix.

MoveIt samples many robot configurations to identify pairs of links that:

- can collide;
- are always in contact;
- are adjacent and do not need to be checked;
- can never collide because of the robot geometry.

This information reduces unnecessary collision checks during planning.

### 5.3 Define a virtual joint

A virtual joint connects the robot base to the world.

For a fixed industrial arm, create a fixed virtual joint between the world frame and the robot base link.

Example:

- joint name: `fixed_base`;
- parent frame: `world`;
- child link: robot base link;
- joint type: `fixed`.

### 5.4 Define the planning group

A planning group defines the joints that MoveIt controls together.

Create a planning group named `arm` and select:

- the `KDLKinematicsPlugin`;
- the first link of the arm as the base link;
- the last link of the arm as the tip link.

The selected chain must include all the joints of the manipulator.

If the robot has a gripper, it can be defined as a separate planning group.

### 5.5 Define named robot poses

Named poses are useful joint configurations such as:

- `home`;
- `ready`;
- `gripper_open`;
- `gripper_closed`.

The joint values must be inside the limits of the robot.

### 5.6 Define the end effector

If the robot has a gripper or another tool, define it as an end effector.

The configuration must specify:

- the end-effector planning group;
- the parent link where the tool is attached;
- the parent arm planning group.

This step is not required for a robot without a gripper.

### 5.7 Configure the MoveIt controllers

MoveIt plans trajectories but does not directly control the motors. It sends the planned trajectory to a `ros2_control` controller.

For the robot arm, the controller normally uses the `FollowJointTrajectory` action.

The action servers available in the system can be checked with:

```bash
ros2 action list
ros2 action info /arm_controller/follow_joint_trajectory -t
```
- click on the Auto Add FollowJointsTrajectory Controllers For Each Planning Group button
- That's it! You have just defined the MoveIt Controllers that will allow the MoveIt2 package to plan and execute the motions on the simulated robot.

The MoveIt controller name, action namespace and joint list must match the controllers used by the simulated or real robot.

### 5.8 Generate the package

Enter the author information, select a destination directory inside the workspace `src` directory and generate the package.

A common package name is:

```text
<robot_name>_moveit_config
```

After generating the package, build and source the workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**Fine tune the generated MoveIt2 package**

You will need to do a couple of modifications in order to have it fully working:
- update to the joint_limits.yaml file with
       - change max_velocityto 100.0 and max_accelerationto 0.0
       - define the joint limits for each joint of the robot arm
              - has_position_limits: true
              - min_position: -3.1415926535897931
              - max_position: 3.1415926535897931
- review the MoveIt Controllers
       - review the controller names and types
       ````bash
       ros2 action list
       ros2 action info /gripper_controller/gripper_cmd -t
       ````
       - The robot arm controller is joint_trajectory_controller and the type is FollowJointTrajectory
       - The gripper controller is gripper_controllerand the type is GripperCommand
       - Modify themoveit_controllers.yamlfile according to the previous names

Having done the proper modifications to your MoveIt2 package, you are now ready to start using it to control the simulated UR3e robotic arm.

## 6. Main files in a MoveIt configuration package

A MoveIt configuration package contains mainly configuration files and launch files. It does not normally contain the application that defines the robot behaviour.

| File | Purpose |
|---|---|
| `*.srdf` | Planning groups, virtual joints, named poses and disabled collision pairs |
| `kinematics.yaml` | Numerical IK solver and solver parameters |
| `joint_limits.yaml` | Velocity, acceleration and additional joint limits |
| `moveit_controllers.yaml` | Controllers used by MoveIt to execute trajectories |
| `ros2_controllers.yaml` | `ros2_control` controller configuration, when included |
| `move_group.launch.py` | Starts the main MoveIt planning node |
| `moveit_rviz.launch.py` | Starts RViz2 with the MoveIt MotionPlanning plugin |
| `demo.launch.py` | Starts a complete demonstration configuration |

The SRDF complements the URDF:

- the **URDF** describes the physical and kinematic robot;
- the **SRDF** describes how MoveIt should use the robot.

## 7. Adapting a package for another robot

The standard method is to generate each package with the Setup Assistant. However, packages for similar robots can also be created by copying a working MoveIt configuration package and adapting it.

In this repository, the first package was generated with the Setup Assistant. The other packages were created from the same structure and then adapted for each robot.

This avoids repeating the complete graphical procedure, but every robot-specific configuration must be checked.

The following items must be reviewed:

| File or element | Information to adapt |
|---|---|
| Robot description | URDF or Xacro file and its arguments |
| SRDF | Link names, joint names, planning groups, named poses and disabled collisions |
| `kinematics.yaml` | Planning-group name, IK plugin and solver parameters |
| `joint_limits.yaml` | Correct limits for every robot joint |
| `moveit_controllers.yaml` | Controller name, action namespace and controlled joints |
| Launch files | Robot description and MoveIt package names |
| RViz configuration | Fixed frame, planning group and visualization options |

Do not copy joint limits or controller names without checking them. Incorrect limits can produce unsafe or unrealistic trajectories, and an incorrect controller configuration prevents trajectory execution.

This repository contains separate MoveIt configuration packages for:

- the PUMA robot: `puma_moveit_config`;
- the UR5e robot: `ur5e_moveit_config`;
- the 6-DOF arm on the mecanum robot: `mecanum_moveit_config`;
- the 5-DOF arm on the mecanum robot: `mecanum_5dof_moveit_config`.

## 8. Test the configuration in simulation

The first test is performed with Gazebo, MoveIt and RViz2 in separate terminals.

### Terminal 1: start the simulated robot

PUMA:

```bash
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py \
  model:=my_arm_puma.urdf.xacro \
  controllers:=gz_controllers.yaml \
  use_gripper:=false
```

UR5e:

```bash
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py \
  model:=my_arm_ur5e.urdf.xacro \
  controllers:=gz_controllers.yaml \
  use_gripper:=false
```

Mecanum robot with the 6-DOF arm:

```bash
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py \
  model:=my_arm_mecanum.urdf.xacro \
  controllers:=gz_controllers_mecanum.yaml \
  use_gripper:=true
```

Mecanum robot with the 5-DOF arm:

```bash
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py \
  model:=my_arm_mecanum_5dof.urdf.xacro \
  controllers:=gz_controllers_mecanum.yaml \
  use_gripper:=true
```

### Terminal 2: start MoveIt

Use the package that corresponds to the simulated robot:

```bash
ros2 launch puma_moveit_config move_group.launch.py use_sim_time:=true
```

```bash
ros2 launch ur5e_moveit_config move_group.launch.py use_sim_time:=true
```

```bash
ros2 launch mecanum_moveit_config move_group.launch.py use_sim_time:=true
```

```bash
ros2 launch mecanum_5dof_moveit_config move_group.launch.py use_sim_time:=true
```

### Terminal 3: start RViz2

Again, use the package that corresponds to the robot:

```bash
ros2 launch puma_moveit_config moveit_rviz.launch.py use_sim_time:=true
```

```bash
ros2 launch ur5e_moveit_config moveit_rviz.launch.py use_sim_time:=true
```

```bash
ros2 launch mecanum_moveit_config moveit_rviz.launch.py use_sim_time:=true
```

```bash
ros2 launch mecanum_5dof_moveit_config moveit_rviz.launch.py use_sim_time:=true
```

In the RViz2 **MotionPlanning** panel:

1. select the correct planning group;
2. move the interactive marker to define a target pose;
3. select **Plan** to calculate a trajectory;
4. inspect the planned movement;
5. select **Execute** to send the trajectory to the controller.

Check that:

- the complete robot model is displayed correctly;
- the current state in RViz2 matches the state in Gazebo;
- MoveIt can calculate a valid plan;
- the simulated robot executes the trajectory;
- no joint-limit or controller errors appear in the terminals.

## 9. What MoveIt does and does not guarantee

MoveIt can reject joint configurations and trajectories that collide with the robot model or with known objects in the Planning Scene.

However:

- collision checking only works with correctly defined collision geometry;
- environmental obstacles must be added to the Planning Scene;
- a valid IK solution does not guarantee that a complete path exists;
- a planned path does not automatically guarantee a safe real-robot movement;
- standard planning does not automatically guarantee that singularities are avoided.

Before using a real robot, always check the robot limits, controller configuration, environment model, velocities and planned trajectory.

## 10. Next step: numerical IK and motion planning

At this point, MoveIt 2 knows:

- the robot kinematic structure;
- the planning group;
- the numerical IK solver;
- the joint limits;
- the valid and invalid self-collisions;
- the trajectory controller used to move the robot.

However, we have not yet explained how a ROS 2 program sends Cartesian targets to MoveIt or how MoveIt plans and executes a complete movement.

In the next document, we will use the `my_arm_motion` package to:

- calculate numerical IK solutions;
- understand the importance of the IK seed;
- plan movements to Cartesian poses;
- execute trajectories in simulation;
- define sequences of poses using YAML files;
- add obstacles to the Planning Scene;
- detect unreachable, invalid or colliding targets;
- study joint limits and singular configurations.

