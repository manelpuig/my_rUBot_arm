#!/usr/bin/env python3
import time
from threading import Thread

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class ArmTestScene(Node):
    """Add or remove a simple box in the MoveIt Planning Scene."""

    def __init__(self):
        super().__init__("arm_test_scene")

        self.declare_parameter("operation", "add")
        self.declare_parameter("object_id", "moveit_test_box")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("box_xyz", [0.0, -0.20, 0.40])
        self.declare_parameter("box_size", [0.18, 0.18, 0.55])
        self.declare_parameter("startup_delay_sec", 1.0)
        self.declare_parameter("service_timeout_sec", 10.0)

        self.operation = str(self.get_parameter("operation").value).lower()
        self.object_id = str(self.get_parameter("object_id").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.box_xyz = [
            float(v) for v in self.get_parameter("box_xyz").value
        ]
        self.box_size = [
            float(v) for v in self.get_parameter("box_size").value
        ]
        self.startup_delay_sec = float(
            self.get_parameter("startup_delay_sec").value
        )
        self.service_timeout_sec = float(
            self.get_parameter("service_timeout_sec").value
        )

        if self.operation not in ("add", "remove"):
            raise ValueError("'operation' must be 'add' or 'remove'.")
        if len(self.box_xyz) != 3:
            raise ValueError("'box_xyz' must contain three values.")
        if len(self.box_size) != 3 or any(v <= 0.0 for v in self.box_size):
            raise ValueError("'box_size' must contain three positive values.")

        self.callback_group = ReentrantCallbackGroup()
        self.client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
            callback_group=self.callback_group,
        )

    def _build_collision_object(self):
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = self.object_id

        if self.operation == "remove":
            collision_object.operation = CollisionObject.REMOVE
            return collision_object

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(self.box_size)

        pose = Pose()
        pose.position.x = self.box_xyz[0]
        pose.position.y = self.box_xyz[1]
        pose.position.z = self.box_xyz[2]
        pose.orientation.w = 1.0

        collision_object.primitives = [primitive]
        collision_object.primitive_poses = [pose]
        collision_object.operation = CollisionObject.ADD
        return collision_object

    def run(self):
        time.sleep(self.startup_delay_sec)

        if not self.client.wait_for_service(
            timeout_sec=self.service_timeout_sec
        ):
            self.get_logger().error(
                "The /apply_planning_scene service is unavailable."
            )
            return False

        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [
            self._build_collision_object()
        ]

        future = self.client.call_async(request)
        deadline = time.monotonic() + self.service_timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                self.get_logger().error(
                    "Planning Scene request timed out."
                )
                return False
            time.sleep(0.05)

        if not future.done() or future.result() is None:
            self.get_logger().error(
                "Planning Scene request returned no response."
            )
            return False

        if not future.result().success:
            self.get_logger().error(
                f"Could not {self.operation} collision object "
                f"'{self.object_id}'."
            )
            return False

        if self.operation == "add":
            self.get_logger().info(
                f"Added '{self.object_id}' in frame '{self.frame_id}': "
                f"center={self.box_xyz} m, size={self.box_size} m."
            )
        else:
            self.get_logger().info(
                f"Removed collision object '{self.object_id}'."
            )
        return True


def main():
    rclpy.init()
    node = ArmTestScene()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received. Shutting down...")
    finally:
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
