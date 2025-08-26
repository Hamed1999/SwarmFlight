#!/usr/bin/env/ python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__("fibonacci_server")
        
        self.action_server = ActionServer(self, Fibonacci, "fibonacci",self.execute_callback)
        self.get_logger().info("Fibonacci Action Server is ready : )")

    async def execute_callback(self, goal_handle):
        order = goal_handle.request.order
        self.get_logger().info(f"Received goal with order: {order}")
        
        feedback_msg = Fibonacci.Feedback()
        sequence = [0, 1]

        for i in range(2, order):
            sequence.append(sequence[i-2] + sequence[i-1])
            feedback_msg.sequence = sequence
            self.get_logger().info(f"Feedback: {feedback_msg.sequence}")
            goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        return result


def main(args = None):
    rclpy.init(args=args)
    server_node = FibonacciActionServer()
    rclpy.spin(node=server_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()   