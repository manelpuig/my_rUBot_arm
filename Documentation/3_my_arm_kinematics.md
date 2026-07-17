# Robot Arm Kinematics

Robot kinematics relates the joint coordinates of the arm to the position and orientation of its tool.

Two spaces are used:

- **joint space**: joint values `q1 ... q6`;
- **Cartesian space**: TCP position and orientation.

## Forward kinematics

Forward kinematics computes the TCP pose from the joint values:

```text
q = [q1, q2, q3, q4, q5, q6]
                 ↓
              FK(q)
                 ↓
       TCP position and orientation
```

The TCP pose is normally expressed relative to `base_link` as:

```text
x, y, z, roll, pitch, yaw
```

## Inverse kinematics

Inverse kinematics performs the opposite operation:

```text
Desired TCP pose
       ↓
Inverse kinematics
       ↓
Joint solution q
```

A robot may have several valid solutions, no solution, or a solution close to a singular configuration.

## Analytical IK of the PUMA arm

The PUMA robot has a spherical wrist. The axes of joints 4, 5 and 6 intersect at the wrist centre.

This allows the problem to be separated into two parts:

1. compute joints 1–3 from the wrist-centre position;
2. compute joints 4–6 from the desired orientation.

The implementation supports:

- `elbow:=up` or `elbow:=down`;
- `wrist:=noflip` or `wrist:=flip`.

## TCP and wrist centre

The desired pose is defined for the TCP frame `tool`, but the first three joints position the wrist centre at `link6`.

```text
p_wrist = p_tcp - R_tcp · p_tool_offset
```

For the current PUMA model:

```text
p_tool_offset = [0, 0, tool_z]
tool_z = 0.15 m
```

Therefore, the tool offset must be considered before solving joints 1–3.

## Run the analytical IK in Gazebo

First start the PUMA simulation:

```bash
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py \
  use_sim_time:=true \
  model:=my_arm_puma.urdf.xacro
```

In another sourced terminal, send a TCP target:

```bash
ros2 launch my_arm_kinematics puma_ikine_pose.launch.py \
  target_xyz:="[0.45,0.10,0.55]" \
  target_rpy_deg:="[0.0,20.0,45.0]" \
  elbow:=up \
  wrist:=noflip
```
![](./Images/Puma_gz_ik_analytic.png)
![](./Images/Puma_gz_ik_analytic_shell.png)

Main launch arguments:

| Argument | Meaning |
|---|---|
| `target_xyz` | TCP position `[x,y,z]` in metres |
| `target_rpy_deg` | TCP orientation `[roll,pitch,yaw]` in degrees |
| `elbow` | `up` or `down` |
| `wrist` | `noflip` or `flip` |
| `tool_z` | distance from `link6` to the TCP |
| `time_sec` | trajectory duration |
| `tcp_frame` | frame used for final verification |
| `use_sim_time` | use the Gazebo clock |

The node converts the TCP target into a wrist-centre target, computes the joint values, sends a trajectory to `/arm_controller/joint_trajectory` and verifies the final TCP pose.

## Verify the TCP pose

```bash
ros2 run tf2_ros tf2_echo base_link tool
```

For repeated targets, Gazebo can remain open. Only the IK launch must be executed again with a new target.
