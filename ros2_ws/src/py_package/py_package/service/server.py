#!/usr/bin/env/ python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts

class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")
        
       
        self.srv = self.create_service(AddTwoInts, "add_two_ints", self.handle_request)
        self.get_logger().info("AddTwoInts Service is ready : )")

    def handle_request(self, request, response):
        response.sum = request.a + request.b
        print(f"Incoming request: {request.a} + {request.b} = {response.sum}")
        return response

def main(args = None):
    rclpy.init(args=args)
    server_node = AddTwoIntsService()
    rclpy.spin(node=server_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()   
