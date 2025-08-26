#!/usr/bin/env/ python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__("fibonacci_client")
       
        self.action_client = ActionClient(self, Fibonacci, "fibonacci")
        self.get_logger().info("Fibonacci Action Client is ready : )")

    def send_goal(self, order):
        self.get_logger().info(f"Sending goal with order: {order}")
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle.accepted:
            self.get_logger().info("Goal was accepted.")
            goal_handle.get_result_async().add_done_callback(self.result_callback)
        else:
            self.get_logger().info("Goal was rejected.")

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result sequence: {result}")


def main(args = None):
    rclpy.init(args=args)
    client_node = FibonacciActionClient()
    client_node.send_goal(order=5)
    rclpy.spin(node=client_node)
    rclpy.shutdown(client_node)


if __name__ == "__main__":
    main()   
