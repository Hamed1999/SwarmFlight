#!/usr/bin/env/ python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__("listener")
        
       
        self.subscription = self.create_subscription(
            String, "chatter", self.news_callback, 10)
        self.get_logger().info("Listener is ready to listen : )")

    def news_callback(self, msg):
        print(f"Received news: {msg.data}")

def main(args = None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()   
