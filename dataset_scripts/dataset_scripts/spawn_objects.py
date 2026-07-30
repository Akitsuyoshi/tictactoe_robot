import os
import math
import random

import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import SpawnEntity


class SpawnObjects(Node):
    def __init__(self):
        super().__init__("spawn_objects")

        self.client = self.create_client(
            SpawnEntity,
            "/spawn_entity"
        )

        self.get_logger().info("Waiting for /spawn_entity service...")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Still waiting...")

        self.model_path = os.path.expanduser(
            "~/ros2_ws/src/piper_ros/models"
        )
        
        self.model_paths = {
            "grid": [],
            "cross": [],
            "circle": []
        }
        model_variations = {
            "grid": ["grid", "thin_grid"],
            "cross": ["cross", "thin_cross"],
            "circle": ["circle", "thin_circle"],
        }

        for model_type, variants in model_variations.items():
            for variant in variants:
                file_path = os.path.join(self.model_path, variant, "model.sdf")

                try:
                    with open(file_path, "r") as f:
                        self.model_paths[model_type].append(f.read())

                    self.get_logger().info(f"Loaded {variant}")

                except Exception as e:
                    self.get_logger().error(
                        f"Failed to load {variant}: {e}"
                    )

        self.positions = [
            (0.245, -0.025),
            (0.200, -0.025),
            (0.155, -0.025),

            (0.245, 0.020),
            (0.200, 0.020),
            (0.155, 0.020),

            (0.245, 0.065),
            (0.200, 0.065),
            (0.155, 0.065),
        ]

        self.declare_parameter("board", "EEEEEEEEE")
        self.declare_parameter("is_train", True)
        self.board = self.get_parameter("board").value
        self.is_train = self.get_parameter("is_train").value

        self.spawn_list = []

        # Spawn grid first
        grid_x = 0.2
        grid_y = 0.02
        grid_x += random.uniform(-0.0005, 0.0005)
        grid_y += random.uniform(-0.0005, 0.0005)
        self.spawn_list.append(
            (
                "grid",
                "grid",
                self.get_random_model("grid"),
                grid_x,
                grid_y,
                0.021
            )
        )

        mapping = {
            "X": "cross",
            "O": "circle"
        }

        # Scale between 0.9 and 1.1
        start = 0.9
        step = 0.01
        count = 21
        scale_options = [round(start + i * step, 2) for i in range(count)]

        for i, cell in enumerate(self.board):
            if cell == "E":
                continue

            model = mapping[cell]

            x, y = self.positions[i]

            # Add small variation for each symble position
            x += random.uniform(-0.002, 0.002)
            y += random.uniform(-0.002, 0.002)

            # Get the base XML layout from our cache
            base_xml = self.get_random_model(model)
            
            # FIX: Randomly select a scale and modify the cached XML string directly
            chosen_scale = random.choice(scale_options)
            scale_string = f"<scale>{chosen_scale} {chosen_scale} {chosen_scale}</scale>"
            
            # Replaces any existing <scale> tags (like <scale>1 1 1</scale>) with the randomized scale
            modified_xml = base_xml.replace("<scale>1 1 1</scale>", scale_string)

            self.spawn_list.append(
                (
                    f"move_{i}",
                    model, 
                    modified_xml,
                    x,
                    y,
                    0.021
                )
            )

        self.current_index = 0

        self.spawn_next()

    def spawn_next(self):

        if self.current_index >= len(self.spawn_list):
            self.get_logger().info("Finished spawning.")
            rclpy.shutdown()
            return

        name, model_type, xml_content, x, y, z = self.spawn_list[self.current_index]

        request = SpawnEntity.Request()

        request.name = name
        request.xml = xml_content
        request.robot_namespace = ""

        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z

        if model_type == "grid":
            # ±3°
            yaw = math.radians(random.uniform(-3.0, 3.0))
        else: # cross/circle
            # ±180°
            yaw = math.radians(random.uniform(-180.0, 180.0))
            

        request.initial_pose.orientation.x = 0.0
        request.initial_pose.orientation.y = 0.0
        request.initial_pose.orientation.z = math.sin(yaw / 2.0)
        request.initial_pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f"Spawning {name}")

        future = self.client.call_async(request)

        future.add_done_callback(self.spawn_response)

    def spawn_response(self, future):
        try:
            response = future.result()

            self.get_logger().info(
                f"{self.spawn_list[self.current_index][0]} "
                f"success={response.success} "
                f"message={response.status_message}"
            )

        except Exception as e:
            self.get_logger().error(str(e))

        self.current_index += 1

        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

        self.spawn_next()
    
    def get_random_model(self, model_type):
        weights = [0, 100]
        if  self.is_train:
            weights = [10, 90]

        return random.choices(self.model_paths[model_type], weights=weights, k=1)[0]

def main(args=None):
    rclpy.init(args=args)
    node = SpawnObjects()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()