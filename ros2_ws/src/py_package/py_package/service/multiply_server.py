#!/usr/bin/env/ python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from my_interfaces.srv import Calculate

class CalculateService(Node):
    def __init__(self):
        super().__init__("multiple_two_ints_server")
        
       
        self.srv = self.create_service(Calculate, "multiple_two_ints", self.handle_request)
        self.get_logger().info("Calculate Service is ready : )")

    def handle_request(self, request, response):
        response.product = request.a * request.b
        print(f"Incoming request: {request.a} * {request.b} = {response.product}")
        return response

def main(args = None):
    rclpy.init(args=args)
    server_node = CalculateService()
    rclpy.spin(node=server_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()   
