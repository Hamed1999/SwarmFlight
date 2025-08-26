#!/usr/bin/env/ python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__("talker")
        
        self.count = 0
        self.publisher_ = self.create_publisher(String, "chatter", 10) # topic name
        self.timer = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Talker node has been started : )")

    def publish_news(self):
        self.count += 1
        msg = String()
        msg.data = f"Hello from talker node for {self.count} times."
        self.get_logger().info(msg.data)
        self.publisher_.publish(msg)


def main(args = None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()   
