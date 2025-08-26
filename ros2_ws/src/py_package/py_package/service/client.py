#!/usr/bin/env/ python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self, a, b):
        super().__init__("add_two_ints_client")
        
       
        self.client = self.create_client(AddTwoInts, "add_two_ints")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not availabe.\nwaiting...")

        self.request = AddTwoInts.Request()
        self.request.a = a
        self.request.b = b

        self.get_logger().info("AddTwoInts Service is ready : )")

    def send_request(self):
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self,future)

        if future.result() :
            self.get_logger().info(f"The result: {future.result().sum}")
        else:
            self.get_logger().info("Service Call Failed : /")


def main(args = None):
    rclpy.init(args=args)
    client_node = AddTwoIntsClient(a=3, b=7)
    client_node.send_request()
    rclpy.shutdown()


if __name__ == "__main__":
    main()   
