#!/usr/bin/env/ python3
import rclpy
from rclpy.node import Node
from my_interfaces.msg import MyMessage

class ListenerNode(Node):
    def __init__(self):
        super().__init__("info_listener")
        
       
        self.subscription = self.create_subscription(
            MyMessage, "info", self.listener_callback, 10)
        self.get_logger().info("Info Listener is ready to listen : )")

    def listener_callback(self, msg):
        print(f"\nName: {msg.first_name} {msg.last_name}\nAge: {msg.age}\nScore: {msg.score}")

def main(args = None):
    rclpy.init(args=args)
    subscriber_node = ListenerNode()
    try:
        rclpy.spin(node=subscriber_node)
    except KeyboardInterrupt:
        subscriber_node.get_logger().info('Shutting down subscriber...')
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()   
