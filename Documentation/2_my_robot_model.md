# Robot model

Once you have created different robot-arm models:
- generic 6DoF robot-arm
- PUMA robot-arm
- UR5e robot-arm
- Mecanum_5dof-arm

<p align="center">
  <img src="./Images/Puma.png" alt="Puma Robot" width="200">
  <img src="./Images/UR5e.png" alt="UR5e Robot" width="250">
  <img src="./Images/my_arm_mecanum_5dof.jpg" alt="Mecanum 5DoF Arm" width="250">
</p>

We can see the different models in **RVIZ2 tool**:
- generic 6DoF robot-arm:
````shell
ros2 launch my_arm_description display.launch.py use_sim_time:=false model:=my_arm.urdf.xacro
````
![](./Images/my_arm_rviz.png)

- PUMA robot-arm:
````shell
ros2 launch my_arm_description display.launch.py use_sim_time:=false model:=my_arm_puma.urdf.xacro
````
![](./Images/my_arm_puma_rviz.png)
- UR5e robot-arm:
````shell
ros2 launch my_arm_description display.launch.py use_sim_time:=false model:=my_arm_ur5e.urdf.xacro
````
![](./Images/my_arm_ur5e_rviz.png)

- Mecanum 5dof-arm:
````shell
ros2 launch my_arm_description display.launch.py use_sim_time:=false model:=my_arm_mecanum_5dof.urdf.xacro
````
![](./Images/my_arm_mecanum_5dof_rviz.png)

**Bringup the robot arm in Gazebo sim**:

- Bringup the robot arm in Gazebo sim:
````shell
ros2 launch my_arm_gz gz_sim.launch.py use_sim_time:=true model:=my_arm_puma.urdf.xacro
````

![](./Images/my_arm_puma2_gz.png)
![](./Images/my_arm_ur5e_gz.png)

- Enviar joint-trajectory
````shell
ros2 launch my_arm_control send_joint_trajectory.launch.py use_sim_time:=true
````
![](./Images/send_joints_puma.png)
![](./Images/send_joints_ur5e.png)
# Move to pose

We consider 2 cases:
- Analitical solution when we have spherical wrist (PUMA)
- Numerical solution when we have not spherical wrist (UR5e)

## Analitical solution

The analytical solution is defined on package `my_arm_kinematics`:
- Launch the simulation environment
````shell
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py use_sim_time:=true model:=my_arm_puma.urdf.xacro
````
- Launch the `send_pose_trajectory` node:
````shell
ros2 launch my_arm_control send_pose_trajectory.launch.py use_sim_time:=true robot_model:=puma
````
![](./Images/send_pose_puma.png)
![](./Images/send_pose_puma_node.png)
![](./Images/send_pose_puma_robodk.png)


## Numerical solution

This node receives a desired **tool pose** (position + orientation) expressed in the **base frame** and computes a 6-joint configuration using **numerical inverse kinematics (IK)**.

## What it does
- Computes `q` such that `FK(q) ≈ T_des` (target pose).
- Uses a **numerical Jacobian** (finite differences) and a **Damped Least Squares** step to update the joints iteratively.
- Once IK converges (or reaches the iteration limit), it sends a `FollowJointTrajectory` goal to the controller:
  `/arm_controller/follow_joint_trajectory`.

- Launch the simulation environment
````shell
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py use_sim_time:=true model:=my_arm_puma.urdf.xacro
````
- Launch the `send_pose_trajectory` node:
````shell
ros2 launch my_arm_control send_pose_trajectory.launch.py use_sim_time:=true robot_model:=puma
````
![](./Images/send_pose_puma.png)

> Requirement! python3 -m pip install "numpy<1.24" --force-reinstall
