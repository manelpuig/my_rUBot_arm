# Set up my arm motion


 Webgraphy:
 - [moveit_install](https://moveit.ai/install-moveit2/binary/)
 - [moveit_Documentation](https://moveit.picknik.ai/humble/index.html)

## Test motion

- Bringup
````bash
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py use_sim_time:=true model:=my_arm_puma.urdf.xacro
ros2 launch my_arm_gazebo my_arm_gazebo.launch.py use_sim_time:=true model:=my_arm_ur5e.urdf.xacro
````
- Launch moveit2
````bash
ros2 launch puma_moveit_config move_group.launch.py use_sim_time:=true
or to obtain rviz2
ros2 launch ur5e_moveit_config move_group_rviz.launch.py use_sim_time:=true
````
- Launch arm_pose.py for puma `handshake`
````bash
ros2 launch my_arm_motion arm_pose.launch.py use_sim_time:=true target_xyz:="[140, -800, 300]" target_rpy:="[0.0, 70.0, -90.0]" seed_from_joint_states:=false seed_joints:="[-90, -40, 30, 0, 0, 0]" execute:=true
````
- or for ur5e
ros2 launch my_arm_motion arm_pose.launch.py use_sim_time:=true target_xyz:="[0, -400, 500]" target_rpy:="[0.0, 0.0, 0.0]" seed_from_joint_states:=false seed_joints:="[-60,-60,-100,-90,-90,0]" execute:=true
````
- For a sequence for puma:
````bash
ros2 launch my_arm_motion arm_pose_sequence.launch.py sequence_file:=puma_handshake.yaml
````
- or for ur5e
ros2 launch my_arm_motion arm_pose_sequence.launch.py sequence_file:=ur5e_handshake.yaml
````
