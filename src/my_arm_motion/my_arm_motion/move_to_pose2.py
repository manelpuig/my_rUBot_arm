#!/usr/bin/env python3

import math
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2
import time

def quaternion_from_rpy(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def main():
    rclpy.init()

    node = Node("move_to_pose_official")

    #node.declare_parameter("use_sim_time", True)

    node.declare_parameter(
        "joint_names",
        ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    )
    node.declare_parameter("group_name", "arm")
    node.declare_parameter("base_link", "base_link")
    node.declare_parameter("end_effector", "puma_tool")

    node.declare_parameter("target_xyz", [0.40, 0.00, 0.30])
    node.declare_parameter("target_rpy", [0.0, 3.14159, 0.0])

    node.declare_parameter("planner_id", "RRTConnectkConfigDefault")
    node.declare_parameter("max_velocity", 0.2)
    node.declare_parameter("max_acceleration", 0.2)

    node.declare_parameter("cartesian", False)
    node.declare_parameter("cartesian_max_step", 0.0025)
    node.declare_parameter("cartesian_fraction_threshold", 0.0)

    joint_names = list(node.get_parameter("joint_names").value)
    group_name = str(node.get_parameter("group_name").value)
    base_link = str(node.get_parameter("base_link").value)
    end_effector = str(node.get_parameter("end_effector").value)

    target_xyz = list(node.get_parameter("target_xyz").value)
    target_rpy = list(node.get_parameter("target_rpy").value)

    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=joint_names,
        base_link_name=base_link,
        end_effector_name=end_effector,
        group_name=group_name,
        callback_group=callback_group,
        use_move_group_action=False,
    )

    moveit2.planner_id = str(node.get_parameter("planner_id").value)
    moveit2.max_velocity = float(node.get_parameter("max_velocity").value)
    moveit2.max_acceleration = float(node.get_parameter("max_acceleration").value)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    #node.create_rate(1.0).sleep()
    time.sleep(1.0)

    position = [float(v) for v in target_xyz]
    quat_xyzw = quaternion_from_rpy(
        float(target_rpy[0]),
        float(target_rpy[1]),
        float(target_rpy[2]),
    )

    cartesian = bool(node.get_parameter("cartesian").value)
    cartesian_max_step = float(node.get_parameter("cartesian_max_step").value)
    cartesian_fraction_threshold = float(
        node.get_parameter("cartesian_fraction_threshold").value
    )

    node.get_logger().info(f"use_sim_time: {node.get_parameter('use_sim_time').value}")
    node.get_logger().info(f"group_name: {group_name}")
    node.get_logger().info(f"base_link: {base_link}")
    node.get_logger().info(f"end_effector: {end_effector}")
    node.get_logger().info(f"target position: {position}")
    node.get_logger().info(f"target quaternion xyzw: {quat_xyzw}")

    moveit2.move_to_pose(
        position=position,
        quat_xyzw=quat_xyzw,
        frame_id=base_link,
        target_link=end_effector,
        tolerance_position=0.01,
        tolerance_orientation=0.1,
        cartesian=False,
    )

    moveit2.wait_until_executed()

    node.get_logger().info("Motion finished.")

    rclpy.shutdown()
    executor_thread.join()