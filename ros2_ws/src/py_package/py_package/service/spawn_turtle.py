#!/usr/bin/env/ python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.srv import Spawn
        
class SpawnTurtleNode(Node):
    def __init__(self):
        super().__init__("spawn_turtle_node")
        
       
        self.client = self.create_client(Spawn, "spawn")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for the spawn service to be availabe ...")

        x = 6.0
        y = 8.0
        theta = 0.5
        turtle_name = "new_turtle1"
        self.spawn_turtle(x, y, theta, turtle_name)

    def spawn_turtle(self, x, y, theta, turtle_name):
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = self.client.call_async(request)
        future.add_done_callback(self.service_response)

    def service_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Successfully spawned turtle {response.name}")
        except Exception as e:
            self.get_logger().info(f"Failed to spawn turtle: {e}")


def main(args = None):
    rclpy.init(args=args)
    client_node = SpawnTurtleNode()
    client_node.spawn_turtle()
    rclpy.shutdown()


if __name__ == "__main__":
    main()   