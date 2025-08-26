#!/usr/bin/env/ python3
import rclpy
from rclpy.node import Node
from my_interfaces.msg import MyMessage

class InfoNode(Node):
    def __init__(self, first_name, last_name, age, score):
        super().__init__("info_publisher")
        
        self.publisher_ = self.create_publisher(MyMessage, "info", 10) # topic name
        self.timer = self.create_timer(0.5, self.publish_info)
        self.first_name = first_name
        self.last_name = last_name
        self.age = age
        self.score = score
        
        self.get_logger().info("Info node has been started : )")


    def publish_info(self):
        msg = MyMessage()
        msg.first_name = self.first_name
        msg.last_name = self.last_name
        msg.age = self.age
        msg.score = self.score

        self.get_logger().info(f"\nFirst Name: {msg.first_name}\nLast Name: {msg.last_name}\nAge: {msg.age}\nScore: {msg.score}")
        self.publisher_.publish(msg)


def main(args = None):
    rclpy.init(args=args)
    publisher_node = InfoNode(first_name="Hamed", last_name="Salmanizadegan", age=25, score=98.0)
    try:
        rclpy.spin(node=publisher_node)
    except KeyboardInterrupt:
        publisher_node.get_logger().info('Shutting down publisher...')
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()   
