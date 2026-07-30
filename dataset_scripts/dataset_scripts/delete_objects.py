import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity


class DeleteObjects(Node):

    REQUEST_TIMEOUT = 3.0
    MAX_RETRIES = 3
    TIMER_PERIOD = 0.1

    def __init__(self):
        super().__init__("delete_objects")

        self.client = self.create_client(DeleteEntity, "/delete_entity")

        self.entities = ["grid"]
        self.entities.extend([f"move_{i}" for i in range(9)])

        self.current_index = 0
        self.retry_count = 0

        self.future = None
        self.request_start_time = None

        self.get_logger().info("Waiting for /delete_entity service...")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Still waiting...")

        self.get_logger().info("Service available.")

        self.timer = self.create_timer(
            self.TIMER_PERIOD,
            self.update
        )
        self.finished = False

    def update(self):
        # Finished
        if self.current_index >= len(self.entities):
            self.get_logger().info("Finished deleting all objects.")
            self.timer.cancel()
            self.finished = True
            return

        # No active request -> send one
        if self.future is None:
            self.send_request()
            return

        # Request finished
        if self.future.done():
            self.handle_response()
            return

        # Timeout check
        elapsed = (
            self.get_clock().now() - self.request_start_time
        ).nanoseconds / 1e9

        if elapsed >= self.REQUEST_TIMEOUT:

            name = self.entities[self.current_index]

            if self.retry_count < self.MAX_RETRIES:

                self.retry_count += 1

                self.get_logger().warning(
                    f"Timeout deleting '{name}'. "
                    f"Retry {self.retry_count}/{self.MAX_RETRIES}"
                )

                self.future = None

            else:

                self.get_logger().error(
                    f"Giving up deleting '{name}'."
                )

                self.future = None
                self.retry_count = 0
                self.current_index += 1

    def send_request(self):

        name = self.entities[self.current_index]

        request = DeleteEntity.Request()
        request.name = name

        self.get_logger().info(f"Deleting '{name}'")

        self.future = self.client.call_async(request)
        self.request_start_time = self.get_clock().now()

    def handle_response(self):

        name = self.entities[self.current_index]

        try:

            response = self.future.result()

            if response.success:

                self.get_logger().info(
                    f"Deleted '{name}'"
                )

            elif "does not exist" in response.status_message.lower():

                self.get_logger().info(
                    f"'{name}' already deleted."
                )

            else:

                self.get_logger().warning(
                    f"{name}: {response.status_message}"
                )

        except Exception as e:

            self.get_logger().error(
                f"Delete failed for '{name}': {e}"
            )

        self.future = None
        self.retry_count = 0
        self.current_index += 1


def main(args=None):
    rclpy.init(args=args)

    node = DeleteObjects()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and not node.finished:
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

    print("EXITING MAIN")


if __name__ == "__main__":
    main()